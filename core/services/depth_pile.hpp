/**
 * @file depth_pile.hpp
 * @brief Depth pile — bounded FIFO with burst counter for satellite TX depth management.
 */

#pragma once

#include <deque>
#include <vector>
#include <optional>
#include <climits>
#include "service_scheduler.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"
#include "debug.hpp"


template<typename T> class DepthPile {
private:
	struct Entry {
		unsigned int burst_counter;
		T data;
	public:
		Entry(T& d, unsigned int c) {
			data = d;
			burst_counter = c;
		}
	};

	std::deque<Entry> m_entry;
	unsigned int m_max_size;
	unsigned int m_retrieve_index;

public:
	DepthPile(unsigned int max_size=24) : m_max_size(max_size), m_retrieve_index(0) {}

	void clear() {
		m_entry.clear();
	}

	/// @brief Resize the cap and evict oldest entries if currently over the new limit.
	/// Eviction is unconditional — entries with non-zero burst_counter (pending retries)
	/// are sacrificed in favor of fresher data. This matches the LoRa SURFACING_BURST
	/// usage where short surfacings cannot finish a full retry burst, and keeping stale
	/// entries in RAM has no value.
	void set_max_size(unsigned int n) {
		if (n == 0) n = 1;  // safety: never allow a zero-cap pile
		m_max_size = n;
		while (m_entry.size() > m_max_size)
			m_entry.pop_front();
	}

	void store(T& e, unsigned int burst_count) {
		m_entry.push_back(Entry(e, burst_count));
		if (m_entry.size() > m_max_size)
			m_entry.pop_front();
		DEBUG_TRACE("DepthPile::store: depth pile has %u/%u entries", m_entry.size(), m_max_size);
	}

	/// @brief Store, but if the last entry matches `pred`, replace it in-place
	/// rather than append. Used for deduplication: e.g. consecutive NO_FIX
	/// GPS entries convey the same information ("still no fix") and the
	/// older one wastes airtime + pile capacity.
	/// @return true if the last entry was replaced; false if a new entry was appended.
	template<typename Pred>
	bool store_or_replace_last(T& e, unsigned int burst_count, Pred pred) {
		if (!m_entry.empty() && pred(m_entry.back().data)) {
			m_entry.back() = Entry(e, burst_count);
			DEBUG_TRACE("DepthPile::store_or_replace_last: replaced last entry, size=%u/%u",
			            m_entry.size(), m_max_size);
			return true;
		}
		store(e, burst_count);
		return false;
	}

	unsigned int size() {
		return m_entry.size();
	}

	unsigned int eligible() {
		unsigned int count = 0;
		for (auto const& it : m_entry) {
			if (it.burst_counter) count++;
		}
		return count;
	}

	/// @brief Remove entries matching a predicate.
	template<typename Pred>
	unsigned int remove_if(Pred pred) {
		unsigned int removed = 0;
		auto it = m_entry.begin();
		while (it != m_entry.end()) {
			if (pred(it->data)) {
				it = m_entry.erase(it);
				removed++;
			} else {
				++it;
			}
		}
		if (removed) {
			m_retrieve_index = 0;
		}
		return removed;
	}

	std::vector<T*> retrieve_latest() {
		std::vector<T*> v;
		if (m_entry.size()) {
			unsigned int idx = m_entry.size()-1;
			if (m_entry[idx].burst_counter)
				v.push_back(&m_entry[idx].data);
		}
		return v;
	}

	// Peek the most recent entry without consuming a burst slot. Unlike
	// retrieve_latest(), returns the entry even if burst_counter == 0 (i.e. all
	// scheduled retries have been used). Intended for read-only inspection by
	// the BaseGnssStrategy::REUSE_LAST path, which needs to read the last fix
	// regardless of whether it still has retries left.
	T* peek_back() {
		if (m_entry.empty()) return nullptr;
		return &m_entry.back().data;
	}

	std::vector<T*> retrieve(unsigned int depth, unsigned int max_messages=3) {
		if (depth == 0) depth = 1;  // guard /0 below on a corrupted depth_pile config (mirrors set_max_size)
		max_messages = std::min(depth, max_messages);
		unsigned int max_index = (depth + (max_messages-1)) / max_messages;
		unsigned int span = std::min(max_messages, (unsigned int)m_entry.size());
		std::vector<T*> v;

		DEBUG_TRACE("DepthPile: retrieve: slot=%u/%u span=%u occupancy=%u", m_retrieve_index % max_index, max_index-1, span, m_entry.size());

		// Find first eligible slot for transmission
		unsigned int max_msg_index = m_retrieve_index + max_index;
		unsigned int retrieve_index = 0;
		unsigned int eligible = 0;
		std::optional<unsigned int> first_eligible;
		while (m_retrieve_index < max_msg_index && !eligible) {
			retrieve_index = m_retrieve_index % max_index;
			// Check to see if any GPS entry has a non-zero burst counter
			for (unsigned int k = 0; k < span; k++) {
				unsigned int idx = m_entry.size() - (span * (retrieve_index+1)) + k;
				if (idx < m_entry.size() && m_entry[idx].burst_counter) {
					eligible++;
					if (!first_eligible.has_value())
						first_eligible = idx;
				}
			}

			m_retrieve_index++;
		}

		if (eligible == 1) {
			DEBUG_TRACE("DepthPile: retrieve: idx=%u burst_counter=%u", first_eligible.value(), m_entry[first_eligible.value()].burst_counter);
			m_entry[first_eligible.value()].burst_counter--;
			v.push_back(&m_entry[first_eligible.value()].data);
		} else if (eligible > 1) {
			for (unsigned int k = 0; k < span; k++) {
				unsigned int idx = m_entry.size() - (span * (retrieve_index+1)) + k;
				// We may have zero burst counter in some entries
				if (idx < m_entry.size() && m_entry[idx].burst_counter) {
					DEBUG_TRACE("DepthPile: retrieve: idx=%u burst_counter=%u", idx, m_entry[idx].burst_counter);
					m_entry[idx].burst_counter--;
					v.push_back(&m_entry[idx].data);
				}
			}
		} else {
			DEBUG_TRACE("DepthPile: retrieve: no eligible entries found");
		}

		return v;
	}
};


class DepthPileManager {
public:
	DepthPileManager();

	void notify_peer_event(ServiceEvent& e);
	void clear() {
		m_gps_depth_pile.clear();
		m_als_depth_pile.clear();
		m_ph_depth_pile.clear();
		m_pressure_depth_pile.clear();
		m_sea_temp_depth_pile.clear();
#if ENABLE_AXL_SENSOR
		m_axl_depth_pile.clear();
#endif
	}
	bool eligible() {
		return m_gps_depth_pile.eligible();
	}

	/// @brief Remove all CloudLocate/Fastloc/NO_FIX entries from GPS depth pile.
	/// Called when a real GPS fix arrives to replace degraded entries.
	unsigned int purge_non_fix_entries() {
		return m_gps_depth_pile.remove_if([](const GPSLogEntry& e) {
			return e.info.event_type == GPSEventType::CLOUDLOCATE ||
			       e.info.event_type == GPSEventType::FASTLOC ||
			       e.info.event_type == GPSEventType::NO_FIX;
		});
	}

	std::vector<GPSLogEntry*> retrieve_gps_latest() {
		return m_gps_depth_pile.retrieve_latest();
	}

	// Peek the most recent GPS entry without consuming a burst slot — backs
	// the BaseGnssStrategy::REUSE_LAST path. Returns nullptr on empty pile.
	GPSLogEntry* peek_gps_latest_any() {
		return m_gps_depth_pile.peek_back();
	}

	std::vector<GPSLogEntry*> retrieve_gps(unsigned int depth_pile) {
		return m_gps_depth_pile.retrieve(depth_pile);
	}

	/// @brief Retrieve GPS entries with an explicit per-slot cap. LoRa uses a higher cap
	/// than Argos (which is fixed at 3 by the LDA2 24-byte budget).
	std::vector<GPSLogEntry*> retrieve_gps(unsigned int depth_pile, unsigned int max_messages) {
		return m_gps_depth_pile.retrieve(depth_pile, max_messages);
	}

	GPSLogEntry* retrieve_gps_single(unsigned int depth_pile) {
		try {
			return m_gps_depth_pile.retrieve(depth_pile, 1).at(0);
		} catch (const std::out_of_range& e) {
			return nullptr;
		}
	}

	ServiceSensorData* retrieve_sensor_single(unsigned int depth_pile, ServiceIdentifier service) {
		try {
			if (service == ServiceIdentifier::ALS_SENSOR) {
				return m_als_depth_pile.retrieve(depth_pile, 1).at(0);
			} else if (service == ServiceIdentifier::PH_SENSOR) {
				return m_ph_depth_pile.retrieve(depth_pile, 1).at(0);
			} else if (service == ServiceIdentifier::PRESSURE_SENSOR) {
				return m_pressure_depth_pile.retrieve(depth_pile, 1).at(0);
			} else if (service == ServiceIdentifier::SEA_TEMP_SENSOR) {
				return m_sea_temp_depth_pile.retrieve(depth_pile, 1).at(0);
			} else if (service == ServiceIdentifier::THERMISTOR_SENSOR) {
				// Thermistor shares sea_temp depth pile slot (mutually exclusive sensors)
				return m_sea_temp_depth_pile.retrieve(depth_pile, 1).at(0);
#if ENABLE_AXL_SENSOR
			} else if (service == ServiceIdentifier::AXL_SENSOR) {
				return m_axl_depth_pile.retrieve(depth_pile, 1).at(0);
#endif
			}
			// Unknown sensor type — return nullptr instead of throwing an enum
			// (ErrorCode is not derived from std::exception so it would escape
			// any narrow catch block and reach terminate()).
			return nullptr;
		} catch (const std::out_of_range& e) {
			// Empty deque: retrieve(...).at(0) throws — treat as "no data".
			return nullptr;
		}
	}

private:
	unsigned int m_sensor_tx_enable = 0;
	unsigned int m_sensor_tx_current = 0;
	Scheduler::TaskHandle m_timeout_task;
	DepthPile<GPSLogEntry> m_gps_depth_pile;
	GPSLogEntry m_gps_cache;
	DepthPile<ServiceSensorData> m_als_depth_pile;
	ServiceSensorData m_als_cache;
	DepthPile<ServiceSensorData> m_pressure_depth_pile;
	ServiceSensorData m_pressure_cache;
	DepthPile<ServiceSensorData> m_ph_depth_pile;
	ServiceSensorData m_ph_cache;
	DepthPile<ServiceSensorData> m_sea_temp_depth_pile;
	ServiceSensorData m_sea_temp_cache;
#if ENABLE_AXL_SENSOR
	DepthPile<ServiceSensorData> m_axl_depth_pile;
	ServiceSensorData m_axl_cache;
#endif

	void update_depth_pile();
};
