/**
 * @file turtle_simulation_1year.cpp
 * @brief Full scientific simulation of a sea turtle's behaviour over one year
 *
 * A realistic simulation of a LinkIt V4 tracker deployed on a sea turtle
 * (Caretta caretta). Produces scientific data with a detailed timeline and full
 * statistics.
 *
 * What is simulated:
 * - dive behaviour based on real biological data
 * - seasonal variation (temperature, migratory behaviour)
 * - sensors: temperature, pressure, pH, salinity, light
 * - GPS with a time-to-fix that varies with conditions
 * - Argos transmissions with pass prediction
 * - surface/water detection (SWS) with the effect of salinity
 *
 * @author Linkit V4 Test Framework
 * @date 2024
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "fake_logger.hpp"
#include "fake_config_store.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"
#include "bsp.hpp"
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <fstream>

extern ConfigurationStore *configuration_store;
extern RTC *rtc;
extern Timer *system_timer;
extern Scheduler *system_scheduler;

// ============================================================================
// CONSTANTES SCIENTIFIQUES
// ============================================================================

namespace Science {
// Constantes physiques
constexpr double GRAVITY = 9.81;                  // m/s²
constexpr double SEAWATER_DENSITY = 1025.0;       // kg/m³
constexpr double PRESSURE_AT_SURFACE = 101325.0;  // Pa (1 atm)

// Paramètres biologiques tortue caouanne (Caretta caretta)
constexpr double TYPICAL_DIVE_DURATION_MIN = 15.0;  // minutes
constexpr double MAX_DIVE_DURATION_MIN = 180.0;     // 3 heures max
constexpr double TYPICAL_DEPTH_M = 25.0;            // mètres
constexpr double MAX_DEPTH_M = 200.0;               // mètres
constexpr double SURFACE_INTERVAL_MIN = 1.0;        // minutes entre plongées
constexpr double SURFACE_INTERVAL_MAX = 30.0;       // minutes

// Vitesses de déplacement
constexpr double DESCENT_RATE_MS = 0.3;       // m/s
constexpr double ASCENT_RATE_MS = 0.25;       // m/s
constexpr double HORIZONTAL_SPEED_KMH = 2.5;  // km/h en migration
}

// ============================================================================
// STRUCTURES DE DONNEES SCIENTIFIQUES
// ============================================================================

struct GeoPosition {
	double latitude;   // degrés décimaux
	double longitude;  // degrés décimaux
	double depth_m;    // mètres sous la surface

	std::string to_string() const {
		std::stringstream ss;
		ss << std::fixed << std::setprecision(4) << (latitude >= 0 ? "N" : "S") << std::abs(latitude) << "° "
		   << (longitude >= 0 ? "E" : "W") << std::abs(longitude) << "°";
		return ss.str();
	}
};

struct SensorReading {
	uint32_t timestamp;    // secondes depuis début simulation
	double temperature_c;  // °C
	double pressure_bar;   // bar
	double salinity_psu;   // PSU
	double ph;             // pH
	double light_lux;      // lux
	uint16_t sws_adc;      // valeur ADC brute
	bool is_underwater;
};

struct GPSFix {
	uint32_t timestamp;
	double latitude;
	double longitude;
	uint32_t ttf_seconds;  // Time To Fix
	uint8_t satellites;
	double hdop;
	bool valid;
};

struct DiveEvent {
	uint32_t start_time;
	uint32_t end_time;
	double max_depth_m;
	double avg_depth_m;
	double bottom_time_s;  // temps au fond
	double descent_rate_ms;
	double ascent_rate_ms;
	double bottom_temp_c;
	std::string dive_type;  // "foraging", "resting", "transit", "exploratory"
};

// Types de messages Argos
enum ArgosMessageType {
	ARGOS_DOPPLER = 0,  // Doppler pur (à chaque remontée surface)
	ARGOS_GPS = 1,      // Position GPS quand fix obtenu
	ARGOS_REGULAR = 2   // Transmission régulière avec données satellites
};

struct ArgosTransmission {
	uint32_t timestamp;
	double latitude;
	double longitude;
	ArgosMessageType message_type;
	uint32_t satellite_counter;  // Compteur incrémental données satellite
	bool success;
	uint32_t pass_duration_s;

	const char *get_type_name() const {
		switch (message_type) {
		case ARGOS_DOPPLER: return "DOPPLER";
		case ARGOS_GPS: return "GPS_FIX";
		case ARGOS_REGULAR: return "REGULAR";
		default: return "UNKNOWN";
		}
	}
};

struct DailyStats {
	uint32_t day_number;
	uint32_t dive_count;
	double total_dive_time_h;
	double total_surface_time_h;
	double max_depth_m;
	double avg_depth_m;
	double avg_dive_duration_min;
	double avg_surface_interval_min;
	double avg_temp_surface_c;
	double avg_temp_deep_c;
	uint32_t gps_fixes;
	uint32_t argos_tx;
	double distance_traveled_km;
	GeoPosition position;
};

struct MonthlyStats {
	uint32_t month;  // 1-12
	std::string month_name;
	uint32_t total_dives;
	double avg_dive_duration_min;
	double max_depth_m;
	double avg_depth_m;
	double surface_time_percent;
	double avg_temp_c;
	double distance_km;
	uint32_t gps_fixes;
	uint32_t argos_tx;
};

// ============================================================================
// MODELE DE TORTUE SCIENTIFIQUE
// ============================================================================

class ScientificTurtleModel {
public:
	enum class BehaviorState {
		SURFACE_BREATHING,
		SURFACE_BASKING,
		DESCENDING,
		FORAGING,
		RESTING_BOTTOM,
		TRANSIT_HORIZONTAL,
		ASCENDING,
		MIGRATING
	};

	enum class Season {
		WINTER,  // Déc-Fév
		SPRING,  // Mar-Mai
		SUMMER,  // Jun-Août
		AUTUMN   // Sep-Nov
	};

private:
	GeoPosition position;
	BehaviorState state;
	Season current_season;
	uint32_t time_in_state;
	uint32_t state_duration;
	double target_depth;
	double vertical_speed;

	// Paramètres de migration
	GeoPosition migration_target;
	bool is_migrating;
	double daily_distance_km;

	// Statistiques courantes
	double current_dive_max_depth;
	double current_dive_depth_sum;
	uint32_t current_dive_samples;

public:
	ScientificTurtleModel() {
		// Position initiale: Côte brésilienne (zone de ponte)
		position.latitude = -8.5;
		position.longitude = -35.0;
		position.depth_m = 0.0;

		state = BehaviorState::SURFACE_BREATHING;
		current_season = Season::SUMMER;
		time_in_state = 0;
		state_duration = generate_surface_duration();
		is_migrating = false;
		daily_distance_km = 0.0;
		current_dive_max_depth = 0.0;
		current_dive_depth_sum = 0.0;
		current_dive_samples = 0;
	}

	void set_day_of_year(uint32_t day) {
		// Mise à jour saison (hémisphère sud)
		if (day >= 335 || day < 60)
			current_season = Season::SUMMER;
		else if (day >= 60 && day < 152)
			current_season = Season::AUTUMN;
		else if (day >= 152 && day < 244)
			current_season = Season::WINTER;
		else
			current_season = Season::SPRING;

		// Comportement migratoire saisonnier
		if (current_season == Season::AUTUMN && !is_migrating) {
			// Migration to the feeding grounds (north)
			is_migrating = true;
			migration_target.latitude = position.latitude + 15.0;
			migration_target.longitude = position.longitude - 5.0;
		} else if (current_season == Season::SPRING && is_migrating) {
			// Retour vers zones de ponte
			is_migrating = true;
			migration_target.latitude = -8.5;
			migration_target.longitude = -35.0;
		}
	}

	uint32_t generate_dive_duration() {
		// Distribution log-normale basée sur données biologiques
		double u = (double)rand() / RAND_MAX;
		double v = (double)rand() / RAND_MAX;
		double z = sqrt(-2.0 * log(u + 0.0001)) * cos(2.0 * M_PI * v);

		double mu = log(Science::TYPICAL_DIVE_DURATION_MIN * 60.0);
		double sigma = 0.8;

		// Ajustement saisonnier
		if (current_season == Season::WINTER) sigma *= 1.2;  // Plus de variabilité
		if (is_migrating) mu -= 0.3;                         // Plongées plus courtes en migration

		uint32_t duration = (uint32_t)exp(mu + sigma * z);
		if (duration < 30) duration = 30;
		if (duration > Science::MAX_DIVE_DURATION_MIN * 60) {
			duration = (uint32_t)(Science::MAX_DIVE_DURATION_MIN * 60);
		}
		return duration;
	}

	uint32_t generate_surface_duration() {
		double base = Science::SURFACE_INTERVAL_MIN * 60.0;
		double range = (Science::SURFACE_INTERVAL_MAX - Science::SURFACE_INTERVAL_MIN) * 60.0;
		double u = (double)rand() / RAND_MAX;
		return (uint32_t)(base + u * u * range);  // Distribution skewed
	}

	double generate_target_depth() {
		double u = (double)rand() / RAND_MAX;
		double v = (double)rand() / RAND_MAX;
		double z = sqrt(-2.0 * log(u + 0.0001)) * cos(2.0 * M_PI * v);

		double mu = log(Science::TYPICAL_DEPTH_M);
		double sigma = 0.6;

		double depth = exp(mu + sigma * z);
		if (depth < 3.0) depth = 3.0;
		if (depth > Science::MAX_DEPTH_M) depth = Science::MAX_DEPTH_M;
		return depth;
	}

	std::string get_dive_type() {
		int r = rand() % 100;
		if (is_migrating) {
			if (r < 60) return "transit";
			if (r < 80) return "resting";
			return "exploratory";
		} else {
			if (r < 50) return "foraging";
			if (r < 75) return "resting";
			if (r < 90) return "exploratory";
			return "transit";
		}
	}

	void update(uint32_t elapsed_seconds) {
		time_in_state += elapsed_seconds;

		switch (state) {
		case BehaviorState::MIGRATING:
		case BehaviorState::SURFACE_BREATHING:
		case BehaviorState::SURFACE_BASKING:
			position.depth_m = 0.0;
			if (time_in_state >= state_duration) {
				state = BehaviorState::DESCENDING;
				time_in_state = 0;
				target_depth = generate_target_depth();
				state_duration = generate_dive_duration();
				vertical_speed = Science::DESCENT_RATE_MS * (0.8 + 0.4 * (double)rand() / RAND_MAX);
				current_dive_max_depth = 0.0;
				current_dive_depth_sum = 0.0;
				current_dive_samples = 0;
			}
			// Déplacement horizontal en surface
			update_horizontal_position(elapsed_seconds * 0.5);
			break;

		case BehaviorState::DESCENDING:
			position.depth_m += vertical_speed * elapsed_seconds;
			if (position.depth_m > current_dive_max_depth) {
				current_dive_max_depth = position.depth_m;
			}
			current_dive_depth_sum += position.depth_m;
			current_dive_samples++;

			if (position.depth_m >= target_depth) {
				position.depth_m = target_depth;
				// Choix du comportement au fond
				int r = rand() % 100;
				if (r < 40)
					state = BehaviorState::FORAGING;
				else if (r < 70)
					state = BehaviorState::RESTING_BOTTOM;
				else
					state = BehaviorState::TRANSIT_HORIZONTAL;
			}
			if (time_in_state >= state_duration) {
				state = BehaviorState::ASCENDING;
				vertical_speed = Science::ASCENT_RATE_MS * (0.8 + 0.4 * (double)rand() / RAND_MAX);
			}
			break;

		case BehaviorState::FORAGING:
		case BehaviorState::RESTING_BOTTOM:
		case BehaviorState::TRANSIT_HORIZONTAL:
			// Légère variation de profondeur
			position.depth_m += ((double)rand() / RAND_MAX - 0.5) * 2.0 * elapsed_seconds * 0.1;
			if (position.depth_m < target_depth - 5) position.depth_m = target_depth - 5;
			if (position.depth_m > target_depth + 5) position.depth_m = target_depth + 5;
			if (position.depth_m < 1) position.depth_m = 1;

			current_dive_depth_sum += position.depth_m;
			current_dive_samples++;

			// Horizontal movement underwater
			if (state == BehaviorState::TRANSIT_HORIZONTAL) {
				update_horizontal_position(elapsed_seconds * 0.3);
			}

			if (time_in_state >= state_duration) {
				state = BehaviorState::ASCENDING;
				vertical_speed = Science::ASCENT_RATE_MS * (0.8 + 0.4 * (double)rand() / RAND_MAX);
			}
			break;

		case BehaviorState::ASCENDING:
			position.depth_m -= vertical_speed * elapsed_seconds;
			current_dive_depth_sum += position.depth_m;
			current_dive_samples++;

			if (position.depth_m <= 0.0) {
				position.depth_m = 0.0;
				state = (rand() % 10 < 2) ? BehaviorState::SURFACE_BASKING : BehaviorState::SURFACE_BREATHING;
				time_in_state = 0;
				state_duration = generate_surface_duration();
			}
			break;

		default: break;
		}
	}

	void update_horizontal_position(double distance_factor) {
		if (is_migrating) {
			// Déplacement vers cible de migration
			double dlat = migration_target.latitude - position.latitude;
			double dlon = migration_target.longitude - position.longitude;
			double dist = sqrt(dlat * dlat + dlon * dlon);

			if (dist > 0.1) {
				double move = distance_factor * Science::HORIZONTAL_SPEED_KMH / 111.0 / 3600.0;
				position.latitude += (dlat / dist) * move;
				position.longitude += (dlon / dist) * move;
				daily_distance_km += move * 111.0;
			} else {
				is_migrating = false;
			}
		} else {
			// Déplacement aléatoire local
			double move = distance_factor * 0.5 / 111.0 / 3600.0;
			position.latitude += ((double)rand() / RAND_MAX - 0.5) * move;
			position.longitude += ((double)rand() / RAND_MAX - 0.5) * move;
			daily_distance_km += move * 111.0 * 0.5;
		}
	}

	bool is_at_surface() const {
		return state == BehaviorState::SURFACE_BREATHING || state == BehaviorState::SURFACE_BASKING;
	}

	const GeoPosition &get_position() const { return position; }
	BehaviorState get_state() const { return state; }
	Season get_season() const { return current_season; }
	double get_current_dive_max_depth() const { return current_dive_max_depth; }
	double get_current_dive_avg_depth() const {
		return current_dive_samples > 0 ? current_dive_depth_sum / current_dive_samples : 0.0;
	}
	double get_daily_distance() const { return daily_distance_km; }
	void reset_daily_distance() { daily_distance_km = 0.0; }

	const char *get_state_name() const {
		switch (state) {
		case BehaviorState::SURFACE_BREATHING: return "SURFACE_BREATHING";
		case BehaviorState::SURFACE_BASKING: return "SURFACE_BASKING";
		case BehaviorState::DESCENDING: return "DESCENDING";
		case BehaviorState::FORAGING: return "FORAGING";
		case BehaviorState::RESTING_BOTTOM: return "RESTING_BOTTOM";
		case BehaviorState::TRANSIT_HORIZONTAL: return "TRANSIT_HORIZONTAL";
		case BehaviorState::ASCENDING: return "ASCENDING";
		case BehaviorState::MIGRATING: return "MIGRATING";
		default: return "UNKNOWN";
		}
	}

	const char *get_season_name() const {
		switch (current_season) {
		case Season::WINTER: return "Hiver";
		case Season::SPRING: return "Printemps";
		case Season::SUMMER: return "Été";
		case Season::AUTUMN: return "Automne";
		default: return "Unknown";
		}
	}
};

// ============================================================================
// MODELE ENVIRONNEMENT OCEANOGRAPHIQUE
// ============================================================================

class OceanEnvironmentModel {
private:
	double base_salinity;
	double salinity_variation;
	uint32_t current_day;

public:
	OceanEnvironmentModel() : base_salinity(35.5), salinity_variation(0.0), current_day(0) {}

	void set_day(uint32_t day) {
		current_day = day;
		// Variation saisonnière de salinité
		salinity_variation = 2.0 * sin(2.0 * M_PI * day / 365.0);
	}

	double get_temperature(double depth, double latitude, uint32_t day_of_year) {
		// Température de surface selon latitude et saison
		double lat_factor = 1.0 - std::abs(latitude) / 90.0;
		double season_factor = cos(2.0 * M_PI * (day_of_year - 172) / 365.0);  // Max en été

		double surface_temp = 18.0 + 10.0 * lat_factor + 4.0 * season_factor;

		// Thermocline
		if (depth < 20) {
			return surface_temp - depth * 0.1;
		} else if (depth < 100) {
			double thermocline_drop = (depth - 20) / 80.0;
			return surface_temp - 2.0 - thermocline_drop * 12.0;
		} else {
			// Eau profonde froide
			return 6.0 + (double)rand() / RAND_MAX * 2.0;
		}
	}

	double get_salinity(double depth, double latitude) {
		double base = base_salinity + salinity_variation;
		// Légère variation avec profondeur
		double depth_factor = depth > 50 ? 0.5 : depth / 100.0;
		// Variation côtière
		double coastal_factor = (std::abs(latitude) < 10) ? -2.0 : 0.0;
		return base + depth_factor + coastal_factor + ((double)rand() / RAND_MAX - 0.5) * 0.2;
	}

	double get_pressure(double depth) {
		// P = P0 + ρgh
		return 1.0 + (Science::SEAWATER_DENSITY * Science::GRAVITY * depth) / 100000.0;
	}

	double get_ph(double depth) {
		// pH falls slightly with depth (rising CO2)
		double base_ph = 8.1;
		double depth_effect = -depth * 0.001;
		return base_ph + depth_effect + ((double)rand() / RAND_MAX - 0.5) * 0.05;
	}

	double get_light(double depth, uint32_t seconds_of_day) {
		// Cycle jour/nuit
		double hour = (seconds_of_day % 86400) / 3600.0;
		double day_factor = sin(M_PI * (hour - 6) / 12.0);
		if (day_factor < 0) day_factor = 0;

		double surface_light = 100000.0 * day_factor;  // lux max à midi

		// Atténuation exponentielle avec profondeur
		double k = 0.05;  // coefficient d'extinction
		return surface_light * exp(-k * depth);
	}

	uint16_t get_sws_adc(double salinity, double /*depth*/, bool underwater) {
		if (!underwater) {
			return (uint16_t)(100 + rand() % 100);  // Air
		}
		// Conductivité proportionnelle à salinité
		double normalized = (salinity - 5.0) / 35.0;
		if (normalized < 0) normalized = 0;
		if (normalized > 1) normalized = 1;
		double base_adc = 800.0 + normalized * 2500.0;
		double noise = ((double)rand() / RAND_MAX - 0.5) * 50.0;
		uint16_t adc = (uint16_t)(base_adc + noise);
		return adc > 16383 ? 16383 : adc;
	}
};

// ============================================================================
// MODELE GPS REALISTE
// ============================================================================

class RealisticGPSModel {
public:
	enum class AcquisitionState { COLD, WARM, HOT };

private:
	AcquisitionState acq_state;
	uint32_t time_since_last_fix;
	uint32_t fix_attempt_time;
	uint32_t current_ttf_target;
	bool has_valid_fix;
	uint8_t satellite_count;
	double hdop;

public:
	RealisticGPSModel()
	    : acq_state(AcquisitionState::COLD),
	      time_since_last_fix(99999),
	      fix_attempt_time(0),
	      current_ttf_target(0),
	      has_valid_fix(false),
	      satellite_count(0),
	      hdop(99.0) {}

	void update(uint32_t elapsed_seconds, bool at_surface) {
		if (!at_surface) {
			time_since_last_fix += elapsed_seconds;
			has_valid_fix = false;
			satellite_count = 0;
			hdop = 99.0;
			fix_attempt_time = 0;
			current_ttf_target = 0;

			// Dégradation état
			if (time_since_last_fix > 7200) acq_state = AcquisitionState::WARM;
			if (time_since_last_fix > 14400) acq_state = AcquisitionState::COLD;
			return;
		}

		fix_attempt_time += elapsed_seconds;

		if (current_ttf_target == 0) {
			current_ttf_target = calculate_ttf();
		}

		if (fix_attempt_time >= current_ttf_target && !has_valid_fix) {
			has_valid_fix = true;
			time_since_last_fix = 0;
			acq_state = AcquisitionState::HOT;
			satellite_count = 6 + rand() % 8;
			hdop = 0.8 + (double)rand() / RAND_MAX * 1.5;
		}
	}

	void reset_session() {
		fix_attempt_time = 0;
		current_ttf_target = 0;
		has_valid_fix = false;
	}

	bool has_fix() const { return has_valid_fix; }
	uint32_t get_ttf() const { return fix_attempt_time; }
	uint8_t get_satellites() const { return satellite_count; }
	double get_hdop() const { return hdop; }
	AcquisitionState get_state() const { return acq_state; }

	const char *get_state_name() const {
		switch (acq_state) {
		case AcquisitionState::COLD: return "COLD";
		case AcquisitionState::WARM: return "WARM";
		case AcquisitionState::HOT: return "HOT";
		default: return "UNKNOWN";
		}
	}

private:
	uint32_t calculate_ttf() {
		uint32_t min_ttf, max_ttf;
		switch (acq_state) {
		case AcquisitionState::HOT:
			min_ttf = 1;
			max_ttf = 5;
			break;
		case AcquisitionState::WARM:
			min_ttf = 15;
			max_ttf = 60;
			break;
		case AcquisitionState::COLD:
		default:
			min_ttf = 45;
			max_ttf = 300;
			break;
		}
		return min_ttf + rand() % (max_ttf - min_ttf + 1);
	}
};

// ============================================================================
// RESULTATS DE SIMULATION COMPLETS
// ============================================================================

struct YearSimulationResults {
	// Métadonnées
	uint32_t simulation_duration_days;
	std::string species;
	std::string study_area;
	GeoPosition start_position;
	GeoPosition end_position;

	// Statistiques globales
	uint32_t total_dives;
	double total_dive_time_hours;
	double total_surface_time_hours;
	double max_depth_m;
	double avg_dive_depth_m;
	double avg_dive_duration_min;
	double total_distance_km;

	// Statistiques capteurs
	double min_temp_c, max_temp_c, avg_temp_c;
	double min_salinity_psu, max_salinity_psu, avg_salinity_psu;
	double min_pressure_bar, max_pressure_bar;
	double min_ph, max_ph;

	// GPS et Argos
	uint32_t total_gps_fixes;
	double avg_ttf_seconds;
	uint32_t cold_starts, warm_starts, hot_starts;
	uint32_t total_argos_transmissions;
	uint32_t argos_doppler_count;
	uint32_t argos_gps_count;
	uint32_t argos_regular_count;

	// SWS
	uint32_t sws_surface_detections;
	uint32_t sws_underwater_detections;

	// Données détaillées
	std::vector<DiveEvent> dives;
	std::vector<GPSFix> gps_fixes;
	std::vector<ArgosTransmission> argos_transmissions;
	std::vector<DailyStats> daily_stats;
	std::vector<MonthlyStats> monthly_stats;

	// Distribution des profondeurs (histogramme)
	std::vector<uint32_t> depth_histogram;  // bins de 10m

	// Distribution des durées de plongée (histogramme)
	std::vector<uint32_t> duration_histogram;  // bins de 5min

	void initialize() {
		depth_histogram.resize(25, 0);     // 0-250m en bins de 10m
		duration_histogram.resize(40, 0);  // 0-200min en bins de 5min
		monthly_stats.resize(12);

		min_temp_c = 999.0;
		max_temp_c = -999.0;
		min_salinity_psu = 999.0;
		max_salinity_psu = 0.0;
		min_pressure_bar = 999.0;
		max_pressure_bar = 0.0;
		min_ph = 999.0;
		max_ph = 0.0;

		// Compteurs Argos
		argos_doppler_count = 0;
		argos_gps_count = 0;
		argos_regular_count = 0;

		const char *months[] = { "Janvier", "Février", "Mars",      "Avril",   "Mai",      "Juin",
		                         "Juillet", "Août",    "Septembre", "Octobre", "Novembre", "Décembre" };
		for (int i = 0; i < 12; i++) {
			monthly_stats[i].month = i + 1;
			monthly_stats[i].month_name = months[i];
		}
	}
};

// ============================================================================
// TEST PRINCIPAL - SIMULATION 1 AN
// ============================================================================

TEST_GROUP(TurtleSimulation1Year) {
	FakeLog *logger;
	ConfigurationStore *fake_config;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;

	ScientificTurtleModel turtle;
	OceanEnvironmentModel ocean;
	RealisticGPSModel gps;
	YearSimulationResults results;

	// État courant
	bool was_at_surface;
	uint32_t current_dive_start;
	DiveEvent current_dive;
	DailyStats current_day_stats;
	uint32_t current_day;
	uint32_t current_month;

	// État Argos
	uint32_t satellite_data_counter;                 // Compteur incrémental données satellite
	uint32_t last_regular_tx_time;                   // Temps dernière TX régulière
	bool gps_tx_sent_this_surface;                   // GPS déjà envoyé cette session surface
	static const uint32_t REGULAR_TX_INTERVAL = 90;  // 90 secondes entre TX régulières

	void setup() {
		logger = new FakeLog("TURTLE_1Y");
		fake_config = new FakeConfigurationStore;
		configuration_store = fake_config;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		srand(42);  // Seed fixe pour reproductibilité

		results.initialize();
		results.species = "Caretta caretta (Tortue caouanne)";
		results.study_area = "Atlantique Sud-Ouest, Brésil";
		results.start_position = turtle.get_position();

		was_at_surface = true;
		current_dive_start = 0;
		current_day = 0;
		current_month = 0;
		current_day_stats = DailyStats{};
		current_dive = DiveEvent{};

		// Initialisation état Argos
		satellite_data_counter = 0;
		last_regular_tx_time = 0;
		gps_tx_sent_this_surface = false;
	}

	void teardown() {
		mock().clear();
		delete logger;
		delete fake_config;
		delete fake_timer;
		delete fake_rtc;
		rtc = nullptr;
	}

	void simulate_timestep(uint32_t total_time, uint32_t step_seconds) {
		uint32_t day_of_year = (total_time / 86400) % 365;
		uint32_t month = day_of_year / 30;
		uint32_t day = total_time / 86400;

		// Mise à jour des modèles
		turtle.set_day_of_year(day_of_year);
		ocean.set_day(day_of_year);

		bool surface_before = turtle.is_at_surface();
		turtle.update(step_seconds);
		gps.update(step_seconds, turtle.is_at_surface());

		const GeoPosition &pos = turtle.get_position();

		// Détection transition surface <-> eau
		if (surface_before && !turtle.is_at_surface()) {
			// Début plongée
			current_dive_start = total_time;
			current_dive.start_time = total_time;
			current_dive.dive_type = turtle.get_state_name();
			results.sws_underwater_detections++;
		} else if (!surface_before && turtle.is_at_surface()) {
			// Fin plongée
			current_dive.end_time = total_time;
			current_dive.max_depth_m = turtle.get_current_dive_max_depth();
			current_dive.avg_depth_m = turtle.get_current_dive_avg_depth();
			current_dive.bottom_time_s = total_time - current_dive_start;
			results.dives.push_back(current_dive);
			results.total_dives++;

			// Histogrammes
			int depth_bin = (int)(current_dive.max_depth_m / 10.0);
			if (depth_bin >= 0 && depth_bin < 25) results.depth_histogram[depth_bin]++;

			int duration_bin = (int)((current_dive.bottom_time_s / 60.0) / 5.0);
			if (duration_bin >= 0 && duration_bin < 40) results.duration_histogram[duration_bin]++;

			// Stats jour
			current_day_stats.dive_count++;
			current_day_stats.total_dive_time_h += current_dive.bottom_time_s / 3600.0;
			if (current_dive.max_depth_m > current_day_stats.max_depth_m) {
				current_day_stats.max_depth_m = current_dive.max_depth_m;
			}

			results.sws_surface_detections++;
			gps.reset_session();
			current_dive = DiveEvent{};

			// === ARGOS DOPPLER à chaque remontée en surface ===
			{
				ArgosTransmission tx;
				tx.timestamp = total_time;
				tx.latitude = pos.latitude;
				tx.longitude = pos.longitude;
				tx.message_type = ARGOS_DOPPLER;
				tx.satellite_counter = satellite_data_counter++;
				tx.success = true;
				results.argos_transmissions.push_back(tx);
				results.total_argos_transmissions++;
				results.argos_doppler_count++;
				current_day_stats.argos_tx++;
			}
			gps_tx_sent_this_surface = false;  // Reset pour nouvelle session surface
		}

		// Capteurs
		double temp = ocean.get_temperature(pos.depth_m, pos.latitude, day_of_year);
		double salinity = ocean.get_salinity(pos.depth_m, pos.latitude);
		double pressure = ocean.get_pressure(pos.depth_m);
		double ph = ocean.get_ph(pos.depth_m);

		if (temp < results.min_temp_c) results.min_temp_c = temp;
		if (temp > results.max_temp_c) results.max_temp_c = temp;
		if (salinity < results.min_salinity_psu) results.min_salinity_psu = salinity;
		if (salinity > results.max_salinity_psu) results.max_salinity_psu = salinity;
		if (pressure < results.min_pressure_bar) results.min_pressure_bar = pressure;
		if (pressure > results.max_pressure_bar) results.max_pressure_bar = pressure;
		if (ph < results.min_ph) results.min_ph = ph;
		if (ph > results.max_ph) results.max_ph = ph;

		// Profondeur max
		if (pos.depth_m > results.max_depth_m) {
			results.max_depth_m = pos.depth_m;
		}

		// GPS fixes
		if (turtle.is_at_surface() && gps.has_fix()) {
			static uint32_t last_fix_time = 0;
			if (total_time - last_fix_time >= 60) {  // Max 1 fix/min
				GPSFix fix;
				fix.timestamp = total_time;
				fix.latitude = pos.latitude;
				fix.longitude = pos.longitude;
				fix.ttf_seconds = gps.get_ttf();
				fix.satellites = gps.get_satellites();
				fix.hdop = gps.get_hdop();
				fix.valid = true;
				results.gps_fixes.push_back(fix);
				results.total_gps_fixes++;
				current_day_stats.gps_fixes++;

				// Comptage par état
				switch (gps.get_state()) {
				case RealisticGPSModel::AcquisitionState::HOT: results.hot_starts++; break;
				case RealisticGPSModel::AcquisitionState::WARM: results.warm_starts++; break;
				case RealisticGPSModel::AcquisitionState::COLD: results.cold_starts++; break;
				default: break;
				}

				last_fix_time = total_time;

				// === Argos GPS TX as soon as a fix lands (once per surface session) ===
				if (!gps_tx_sent_this_surface) {
					ArgosTransmission tx;
					tx.timestamp = total_time;
					tx.latitude = pos.latitude;
					tx.longitude = pos.longitude;
					tx.message_type = ARGOS_GPS;
					tx.satellite_counter = satellite_data_counter++;
					tx.success = true;
					results.argos_transmissions.push_back(tx);
					results.total_argos_transmissions++;
					results.argos_gps_count++;
					current_day_stats.argos_tx++;
					gps_tx_sent_this_surface = true;
					last_regular_tx_time = total_time;  // Reset timer TX régulières
				}
			}
		}

		// === Regular Argos TX with an incrementing counter (every 90 s at surface) ===
		if (turtle.is_at_surface() && (total_time - last_regular_tx_time >= REGULAR_TX_INTERVAL)) {
			ArgosTransmission tx;
			tx.timestamp = total_time;
			tx.latitude = pos.latitude;
			tx.longitude = pos.longitude;
			tx.message_type = ARGOS_REGULAR;
			tx.satellite_counter = satellite_data_counter++;
			tx.success = true;
			results.argos_transmissions.push_back(tx);
			results.total_argos_transmissions++;
			results.argos_regular_count++;
			current_day_stats.argos_tx++;
			last_regular_tx_time = total_time;
		}

		// Comptage temps surface/eau
		if (turtle.is_at_surface()) {
			results.total_surface_time_hours += step_seconds / 3600.0;
			current_day_stats.total_surface_time_h += step_seconds / 3600.0;
		} else {
			results.total_dive_time_hours += step_seconds / 3600.0;
		}

		// Changement de jour
		if (day > current_day) {
			current_day_stats.day_number = current_day;
			current_day_stats.position = pos;
			current_day_stats.distance_traveled_km = turtle.get_daily_distance();
			results.total_distance_km += turtle.get_daily_distance();
			results.daily_stats.push_back(current_day_stats);

			// Stats mensuelles
			if (month < 12) {
				results.monthly_stats[month].total_dives += current_day_stats.dive_count;
				results.monthly_stats[month].gps_fixes += current_day_stats.gps_fixes;
				results.monthly_stats[month].argos_tx += current_day_stats.argos_tx;
				results.monthly_stats[month].distance_km += current_day_stats.distance_traveled_km;
				if (current_day_stats.max_depth_m > results.monthly_stats[month].max_depth_m) {
					results.monthly_stats[month].max_depth_m = current_day_stats.max_depth_m;
				}
			}

			turtle.reset_daily_distance();
			current_day_stats = DailyStats{};
			current_day = day;

			// Nouveau mois
			if (month > current_month) {
				current_month = month;
			}
		}
	}

	void print_scientific_report() {
		printf("\n");
		printf("╔══════════════════════════════════════════════════════════════════════════════╗\n");
		printf("║                    RAPPORT SCIENTIFIQUE - SIMULATION 1 AN                    ║\n");
		printf("║                   Suivi Telemetrique Tortue Marine Linkit V4                 ║\n");
		printf("╚══════════════════════════════════════════════════════════════════════════════╝\n");

		printf("\n┌─ INFORMATIONS GENERALES ─────────────────────────────────────────────────────┐\n");
		printf("│ Espèce: %s\n", results.species.c_str());
		printf("│ Zone d'étude: %s\n", results.study_area.c_str());
		printf("│ Durée simulation: %u jours (%.1f mois)\n", results.simulation_duration_days,
		       results.simulation_duration_days / 30.0);
		printf("│ Position départ: %s\n", results.start_position.to_string().c_str());
		printf("│ Position fin: %s\n", results.end_position.to_string().c_str());
		printf("│ Distance totale parcourue: %.1f km\n", results.total_distance_km);
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n┌─ COMPORTEMENT DE PLONGEE ────────────────────────────────────────────────────┐\n");
		printf("│ Nombre total de plongées: %u\n", results.total_dives);
		printf("│ Temps total en plongée: %.1f heures (%.1f%%)\n", results.total_dive_time_hours,
		       100.0 * results.total_dive_time_hours
		           / (results.total_dive_time_hours + results.total_surface_time_hours));
		printf("│ Temps total en surface: %.1f heures (%.1f%%)\n", results.total_surface_time_hours,
		       100.0 * results.total_surface_time_hours
		           / (results.total_dive_time_hours + results.total_surface_time_hours));
		printf("│ Profondeur maximale: %.1f m\n", results.max_depth_m);
		printf("│ Plongées/jour (moyenne): %.1f\n", (double)results.total_dives / results.simulation_duration_days);
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n┌─ DISTRIBUTION DES PROFONDEURS ──────────────────────────────────────────────┐\n");
		printf("│ Profondeur (m)  │ Nb plongées │ Histogramme\n");
		printf("│─────────────────┼─────────────┼──────────────────────────────────────────────\n");
		uint32_t max_count = *std::max_element(results.depth_histogram.begin(), results.depth_histogram.end());
		for (size_t i = 0; i < results.depth_histogram.size() && i < 15; i++) {
			if (results.depth_histogram[i] > 0) {
				int bar_len = (int)(40.0 * results.depth_histogram[i] / max_count);
				std::string bar(bar_len, '#');
				printf("| %3zu - %3zu m     | %8u    | %s\n", i * 10, (i + 1) * 10, results.depth_histogram[i],
				       bar.c_str());
			}
		}
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n┌─ PARAMETRES ENVIRONNEMENTAUX ───────────────────────────────────────────────┐\n");
		printf("│ Température:  %.1f - %.1f °C\n", results.min_temp_c, results.max_temp_c);
		printf("│ Salinité:     %.1f - %.1f PSU\n", results.min_salinity_psu, results.max_salinity_psu);
		printf("│ Pression:     %.1f - %.1f bar\n", results.min_pressure_bar, results.max_pressure_bar);
		printf("│ pH:           %.2f - %.2f\n", results.min_ph, results.max_ph);
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n┌─ TELEMETRIE GPS ─────────────────────────────────────────────────────────────┐\n");
		printf("│ Fixes GPS totaux: %u\n", results.total_gps_fixes);
		printf("│ Cold starts: %u (TTF ~45-300s)\n", results.cold_starts);
		printf("│ Warm starts: %u (TTF ~15-60s)\n", results.warm_starts);
		printf("│ Hot starts:  %u (TTF ~1-5s)\n", results.hot_starts);
		printf("│ Fixes/jour (moyenne): %.1f\n", (double)results.total_gps_fixes / results.simulation_duration_days);
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n┌─ TRANSMISSIONS ARGOS ────────────────────────────────────────────────────────┐\n");
		printf("│ Transmissions totales: %u\n", results.total_argos_transmissions);
		printf("│   - Doppler (remontée surface): %u\n", results.argos_doppler_count);
		printf("│   - GPS (fix obtenu):           %u\n", results.argos_gps_count);
		printf("│   - Régulières (compteur sat):  %u\n", results.argos_regular_count);
		printf("│ Transmissions/jour (moyenne): %.1f\n",
		       (double)results.total_argos_transmissions / results.simulation_duration_days);
		printf("│ Compteur satellite final: %u\n",
		       results.argos_transmissions.empty() ? 0 : results.argos_transmissions.back().satellite_counter);
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n┌─ DETECTION SURFACE/EAU (SWS) ───────────────────────────────────────────────┐\n");
		printf("│ Détections surface: %u\n", results.sws_surface_detections);
		printf("│ Détections sous-marines: %u\n", results.sws_underwater_detections);
		printf("│ Transitions totales: %u\n", results.sws_surface_detections + results.sws_underwater_detections);
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n┌─ STATISTIQUES MENSUELLES ───────────────────────────────────────────────────┐\n");
		printf("│ Mois        │ Plongées │ Prof.max │ GPS fixes │ Argos TX │ Distance\n");
		printf("│─────────────┼──────────┼──────────┼───────────┼──────────┼──────────\n");
		for (size_t i = 0; i < 12; i++) {
			if (results.monthly_stats[i].total_dives > 0) {
				printf("│ %-11s │ %8u │ %6.1f m │ %9u │ %8u │ %6.1f km\n", results.monthly_stats[i].month_name.c_str(),
				       results.monthly_stats[i].total_dives, results.monthly_stats[i].max_depth_m,
				       results.monthly_stats[i].gps_fixes, results.monthly_stats[i].argos_tx,
				       results.monthly_stats[i].distance_km);
			}
		}
		printf("└──────────────────────────────────────────────────────────────────────────────┘\n");

		printf("\n═══════════════════════════════════════════════════════════════════════════════\n");
		printf("                           FIN DU RAPPORT SCIENTIFIQUE\n");
		printf("═══════════════════════════════════════════════════════════════════════════════\n\n");
	}
};

TEST(TurtleSimulation1Year, FullYearSimulation) {
	const uint32_t SIMULATION_DAYS = 365;
	const uint32_t SIMULATION_DURATION = SIMULATION_DAYS * 86400;
	const uint32_t TIME_STEP = 60;  // 1 minute

	results.simulation_duration_days = SIMULATION_DAYS;

	uint32_t total_time = 0;
	while (total_time < SIMULATION_DURATION) {
		simulate_timestep(total_time, TIME_STEP);
		total_time += TIME_STEP;

		// Progress (every 30 days)
		if (total_time % (30 * 86400) == 0) {
			uint32_t month = total_time / (30 * 86400);
			printf("Simulation mois %u/12... Plongées: %u, GPS: %u\n", month, results.total_dives,
			       results.total_gps_fixes);
		}
	}

	results.end_position = turtle.get_position();

	// Afficher rapport scientifique
	print_scientific_report();

	// Vérifications
	CHECK(results.total_dives >= 3000);              // ~8-15 plongées/jour
	CHECK(results.max_depth_m >= 100.0);             // Au moins 100m
	CHECK(results.total_gps_fixes >= 5000);          // GPS fixes réguliers
	CHECK(results.total_argos_transmissions >= 10);  // ~1-3 TX/mois typical for wildlife tracker

	printf("TEST PASSED: Simulation 1 an complète\n");
}
