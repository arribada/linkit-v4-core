/**
 * @file turtle_simulation_test.cpp
 * @brief Simulation complète du comportement d'une tortue marine sur longue durée
 *
 * Ce test simule le comportement réaliste d'un tracker sur tortue marine:
 * - Plongées aléatoires avec durées variables (10s à 2h)
 * - Remontées en surface avec durées variables (30s à 30min)
 * - Variation de la salinité affectant le capteur SWS
 * - Capteurs: température, pression, pH, luminosité, accéléromètre
 * - GPS Time-To-Fix variable selon conditions
 * - Transmissions Argos en surface
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

extern ConfigurationStore *configuration_store;
extern RTC *rtc;
extern Timer *system_timer;
extern Scheduler *system_scheduler;

// ============================================================================
// MODELES DE SIMULATION
// ============================================================================

/**
 * @brief Modèle de comportement de tortue marine
 */
class TurtleBehaviorModel {
public:
    static constexpr uint32_t MIN_DIVE_DURATION = 10;
    static constexpr uint32_t MAX_DIVE_DURATION = 7200;
    static constexpr uint32_t TYPICAL_DIVE_DURATION = 600;
    static constexpr uint32_t MIN_SURFACE_DURATION = 30;
    static constexpr uint32_t MAX_SURFACE_DURATION = 1800;
    static constexpr uint32_t TYPICAL_SURFACE_DURATION = 180;
    static constexpr double MIN_DEPTH = 0.0;
    static constexpr double MAX_DEPTH = 300.0;
    static constexpr double TYPICAL_DEPTH = 30.0;

    enum class State { SURFACE, DIVING, ASCENDING, FEEDING, RESTING };

    State current_state = State::SURFACE;
    uint32_t time_in_state = 0;
    uint32_t state_duration = 0;
    double current_depth = 0.0;
    double target_depth = 0.0;
    double vertical_speed = 0.5;
    double latitude = -4.5;
    double longitude = -35.0;

    uint32_t generate_dive_duration() {
        double u = (double)rand() / RAND_MAX;
        double v = (double)rand() / RAND_MAX;
        double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
        double mu = log((double)TYPICAL_DIVE_DURATION);
        double sigma = 1.0;
        uint32_t duration = (uint32_t)exp(mu + sigma * z);
        if (duration < MIN_DIVE_DURATION) duration = MIN_DIVE_DURATION;
        if (duration > MAX_DIVE_DURATION) duration = MAX_DIVE_DURATION;
        return duration;
    }

    uint32_t generate_surface_duration() {
        double u = (double)rand() / RAND_MAX;
        double v = (double)rand() / RAND_MAX;
        double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
        double mu = log((double)TYPICAL_SURFACE_DURATION);
        double sigma = 0.8;
        uint32_t duration = (uint32_t)exp(mu + sigma * z);
        if (duration < MIN_SURFACE_DURATION) duration = MIN_SURFACE_DURATION;
        if (duration > MAX_SURFACE_DURATION) duration = MAX_SURFACE_DURATION;
        return duration;
    }

    double generate_target_depth() {
        double u = (double)rand() / RAND_MAX;
        double v = (double)rand() / RAND_MAX;
        double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
        double mu = log(TYPICAL_DEPTH);
        double sigma = 0.7;
        double depth = exp(mu + sigma * z);
        if (depth < 1.0) depth = 1.0;
        if (depth > MAX_DEPTH) depth = MAX_DEPTH;
        return depth;
    }

    void update(uint32_t elapsed_seconds) {
        time_in_state += elapsed_seconds;
        switch (current_state) {
            default:
            case State::SURFACE:
                current_depth = 0.0;
                if (time_in_state >= state_duration) {
                    current_state = State::DIVING;
                    time_in_state = 0;
                    state_duration = generate_dive_duration();
                    target_depth = generate_target_depth();
                    vertical_speed = 0.3 + (double)rand() / RAND_MAX * 0.7;
                }
                break;
            case State::DIVING:
                current_depth += vertical_speed * elapsed_seconds;
                if (current_depth >= target_depth) {
                    current_depth = target_depth;
                    current_state = (rand() % 2) ? State::FEEDING : State::RESTING;
                }
                if (time_in_state >= state_duration) {
                    current_state = State::ASCENDING;
                    vertical_speed = 0.2 + (double)rand() / RAND_MAX * 0.5;
                }
                break;
            case State::FEEDING:
            case State::RESTING:
                current_depth += ((double)rand() / RAND_MAX - 0.5) * 2.0;
                if (current_depth < 1.0) current_depth = 1.0;
                if (current_depth > MAX_DEPTH) current_depth = MAX_DEPTH;
                if (time_in_state >= state_duration) {
                    current_state = State::ASCENDING;
                    vertical_speed = 0.2 + (double)rand() / RAND_MAX * 0.5;
                }
                break;
            case State::ASCENDING:
                current_depth -= vertical_speed * elapsed_seconds;
                if (current_depth <= 0.0) {
                    current_depth = 0.0;
                    current_state = State::SURFACE;
                    time_in_state = 0;
                    state_duration = generate_surface_duration();
                    latitude += ((double)rand() / RAND_MAX - 0.5) * 0.001;
                    longitude += ((double)rand() / RAND_MAX - 0.5) * 0.001;
                }
                break;
        }
    }

    bool is_at_surface() const { return current_state == State::SURFACE; }
    const char* get_state_name() const {
        switch (current_state) {
            case State::SURFACE: return "SURFACE";
            case State::DIVING: return "DIVING";
            case State::ASCENDING: return "ASCENDING";
            case State::FEEDING: return "FEEDING";
            case State::RESTING: return "RESTING";
            default: return "UNKNOWN";
        }
    }
};

/**
 * @brief Modèle d'environnement marin
 */
class MarineEnvironmentModel {
public:
    double current_salinity = 35.0;
    double salinity_drift = 0.0;
    double base_salinity = 35.0;

    uint16_t calculate_sws_adc(double salinity, double depth, bool in_water) {
        if (!in_water || depth <= 0.0) return (uint16_t)(100 + rand() % 100);
        double normalized_salinity = (salinity - 5.0) / 35.0;
        if (normalized_salinity < 0) normalized_salinity = 0;
        if (normalized_salinity > 1) normalized_salinity = 1;
        double base_adc = 800.0 + normalized_salinity * 2500.0;
        double noise = ((double)rand() / RAND_MAX - 0.5) * 100.0;
        uint16_t adc = (uint16_t)(base_adc + noise);
        if (adc > 16383) adc = 16383;
        return adc;
    }

    double calculate_water_temperature(double depth) {
        if (depth <= 0) return 20.0 + (double)rand() / RAND_MAX * 10.0;
        if (depth < 50.0) {
            double ratio = depth / 50.0;
            return 30.0 - ratio * 15.0;
        }
        return 4.0 + (double)rand() / RAND_MAX * 11.0;
    }

    double calculate_pressure(double depth) { return 1.0 + depth / 10.0; }
    double calculate_ph(double depth) {
        double base_ph = 8.1;
        double depth_effect = -depth * 0.001;
        double noise = ((double)rand() / RAND_MAX - 0.5) * 0.1;
        return base_ph + depth_effect + noise;
    }

    double calculate_light(double depth, uint32_t time_of_day_seconds) {
        double day_cycle = sin(2.0 * M_PI * time_of_day_seconds / 86400.0);
        double surface_light = (day_cycle > 0) ? 10000.0 * day_cycle : 0.0;
        return surface_light * exp(-0.05 * depth);
    }

    void update_salinity(uint32_t elapsed_seconds) {
        salinity_drift += ((double)rand() / RAND_MAX - 0.5) * 0.001 * elapsed_seconds;
        if (salinity_drift > 10.0) salinity_drift = 10.0;
        if (salinity_drift < -10.0) salinity_drift = -10.0;
        current_salinity = base_salinity + salinity_drift;
        if (rand() % 10000 == 0) {
            int zone = rand() % 3;
            if (zone == 0) base_salinity = 33.0 + (double)rand() / RAND_MAX * 4.0;
            else if (zone == 1) base_salinity = 25.0 + (double)rand() / RAND_MAX * 10.0;
            else base_salinity = 5.0 + (double)rand() / RAND_MAX * 20.0;
            salinity_drift = 0.0;
        }
    }
};

/**
 * @brief Modèle GPS avec Time-To-Fix réaliste
 */
class GPSModel {
public:
    enum class FixState { HOT, WARM, COLD };
    FixState fix_state = FixState::COLD;
    uint32_t time_since_last_fix = 0;
    uint32_t satellite_count = 0;
    double hdop = 99.9;
    bool has_fix = false;
    uint32_t fix_attempt_time = 0;
    uint32_t ttf_target = 0;

    void update(uint32_t elapsed_seconds, bool is_at_surface) {
        if (!is_at_surface) {
            time_since_last_fix += elapsed_seconds;
            has_fix = false;
            satellite_count = 0;
            hdop = 99.9;
            fix_attempt_time = 0;
            if (time_since_last_fix > 3600) fix_state = FixState::WARM;
            if (time_since_last_fix > 14400) fix_state = FixState::COLD;
            return;
        }
        fix_attempt_time += elapsed_seconds;
        if (ttf_target == 0) ttf_target = calculate_ttf();
        if (fix_attempt_time >= ttf_target) {
            has_fix = true;
            time_since_last_fix = 0;
            fix_state = FixState::HOT;
            satellite_count = 6 + rand() % 8;
            hdop = 0.8 + (double)rand() / RAND_MAX * 2.0;
        } else {
            double progress = (double)fix_attempt_time / ttf_target;
            satellite_count = (uint32_t)(progress * 8);
            hdop = 99.9 - progress * 97.0;
            if (hdop < 2.0) hdop = 2.0 + (double)rand() / RAND_MAX;
        }
    }

    void start_new_session() { fix_attempt_time = 0; ttf_target = 0; has_fix = false; }

private:
    uint32_t calculate_ttf() {
        uint32_t min_ttf, max_ttf;
        switch (fix_state) {
            case FixState::HOT: min_ttf = 1; max_ttf = 3; break;
            case FixState::WARM: min_ttf = 10; max_ttf = 45; break;
            case FixState::COLD:
            default: min_ttf = 30; max_ttf = 300; break;
        }
        return min_ttf + rand() % (max_ttf - min_ttf + 1);
    }
};

struct SimulationResults {
    uint32_t total_time_seconds = 0;
    uint32_t total_surface_time = 0;
    uint32_t total_underwater_time = 0;
    uint32_t dive_count = 0;
    uint32_t surface_count = 0;
    double max_depth = 0.0;
    double min_salinity = 999.0;
    double max_salinity = 0.0;
    uint32_t gps_fix_count = 0;
    double avg_ttf = 0.0;
    uint32_t argos_tx_count = 0;
    double min_temp = 999.0, max_temp = -999.0;
    double min_pressure = 999.0, max_pressure = 0.0;
    double min_ph = 999.0, max_ph = 0.0;
    double min_light = 999999.0, max_light = 0.0;
    uint32_t sws_transitions = 0;
    std::vector<std::string> event_log;

    void log_event(uint32_t time, const std::string& event) {
        std::stringstream ss;
        ss << "[" << time / 3600 << "h" << (time % 3600) / 60 << "m] " << event;
        event_log.push_back(ss.str());
        if (event_log.size() > 1000) event_log.erase(event_log.begin());
    }
};

TEST_GROUP(TurtleSimulation)
{
    FakeLog *logger;
    ConfigurationStore *fake_config;
    FakeRTC *fake_rtc;
    FakeTimer *fake_timer;
    TurtleBehaviorModel turtle;
    MarineEnvironmentModel environment;
    GPSModel gps;
    SimulationResults results;

    void setup() {
        logger = new FakeLog("TURTLE_SIM");
        fake_config = new FakeConfigurationStore;
        configuration_store = fake_config;
        fake_rtc = new FakeRTC;
        rtc = fake_rtc;
        fake_timer = new FakeTimer;
        system_timer = fake_timer;
        system_scheduler = new Scheduler(system_timer);
        srand(12345);
    }

    void teardown() {
        mock().clear();
        delete logger;
        delete fake_config;
        delete fake_timer;
        delete fake_rtc;
        rtc = nullptr;  // null the global so a later group's UW event can't deref freed rtc
    }

    void simulate_timestep(uint32_t step_seconds) {
        bool was_at_surface = turtle.is_at_surface();
        turtle.update(step_seconds);
        environment.update_salinity(step_seconds);
        gps.update(step_seconds, turtle.is_at_surface());

        if (was_at_surface != turtle.is_at_surface()) {
            results.sws_transitions++;
            if (turtle.is_at_surface()) {
                results.surface_count++;
                gps.start_new_session();
            } else {
                results.dive_count++;
            }
        }

        results.total_time_seconds += step_seconds;
        if (turtle.is_at_surface()) {
            results.total_surface_time += step_seconds;
            if (gps.has_fix) {
                results.gps_fix_count++;
                results.avg_ttf = (results.avg_ttf * (results.gps_fix_count - 1) + gps.fix_attempt_time) / results.gps_fix_count;
            }
        } else {
            results.total_underwater_time += step_seconds;
            if (turtle.current_depth > results.max_depth) results.max_depth = turtle.current_depth;
        }

        double temp = environment.calculate_water_temperature(turtle.current_depth);
        double pressure = environment.calculate_pressure(turtle.current_depth);
        double ph = environment.calculate_ph(turtle.current_depth);
        double light = environment.calculate_light(turtle.current_depth, results.total_time_seconds % 86400);

        if (temp < results.min_temp) results.min_temp = temp;
        if (temp > results.max_temp) results.max_temp = temp;
        if (pressure < results.min_pressure) results.min_pressure = pressure;
        if (pressure > results.max_pressure) results.max_pressure = pressure;
        if (ph < results.min_ph) results.min_ph = ph;
        if (ph > results.max_ph) results.max_ph = ph;
        if (light < results.min_light) results.min_light = light;
        if (light > results.max_light) results.max_light = light;

        double salinity = environment.current_salinity;
        if (salinity < results.min_salinity) results.min_salinity = salinity;
        if (salinity > results.max_salinity) results.max_salinity = salinity;

        // Transmission Argos toutes les ~90 secondes en surface avec fix GPS
        if (turtle.is_at_surface() && gps.has_fix && rand() % 90 == 0) {
            results.argos_tx_count++;
        }
    }
};

TEST(TurtleSimulation, ShortSimulation1Hour)
{
    const uint32_t SIMULATION_DURATION = 3600;
    const uint32_t TIME_STEP = 1;
    turtle.current_state = TurtleBehaviorModel::State::SURFACE;
    turtle.state_duration = turtle.generate_surface_duration();

    while (results.total_time_seconds < SIMULATION_DURATION) simulate_timestep(TIME_STEP);

    CHECK(results.dive_count >= 1);
    CHECK(results.surface_count >= 1);
    CHECK(results.max_depth > 0);
    CHECK(results.sws_transitions >= 2);

    printf("\n=== Simulation 1 heure ===\n");
    printf("Plongees: %u, Remontees: %u\n", results.dive_count, results.surface_count);
    printf("Profondeur max: %.1f m\n", results.max_depth);
    printf("Temps surface: %u s, Temps sous l'eau: %u s\n", results.total_surface_time, results.total_underwater_time);
    printf("Transitions SWS: %u\n", results.sws_transitions);
}

TEST(TurtleSimulation, MediumSimulation24Hours)
{
    const uint32_t SIMULATION_DURATION = 86400;
    const uint32_t TIME_STEP = 10;
    turtle.current_state = TurtleBehaviorModel::State::SURFACE;
    turtle.state_duration = turtle.generate_surface_duration();

    while (results.total_time_seconds < SIMULATION_DURATION) simulate_timestep(TIME_STEP);

    CHECK(results.dive_count >= 10);
    CHECK(results.max_depth > 10.0);
    CHECK(results.gps_fix_count > 0);
    CHECK(results.sws_transitions >= 20);
    CHECK(results.max_salinity > results.min_salinity);

    printf("\n=== Simulation 24 heures ===\n");
    printf("Plongees: %u, Remontees: %u\n", results.dive_count, results.surface_count);
    printf("Profondeur max: %.1f m\n", results.max_depth);
    printf("Salinite: %.1f - %.1f PSU\n", results.min_salinity, results.max_salinity);
    printf("GPS fixes: %u, TTF moyen: %.1f s\n", results.gps_fix_count, results.avg_ttf);
    printf("Transmissions Argos: %u\n", results.argos_tx_count);
    printf("Temperature: %.1f - %.1f C\n", results.min_temp, results.max_temp);
    printf("Pression: %.1f - %.1f bar\n", results.min_pressure, results.max_pressure);
}

TEST(TurtleSimulation, LongSimulation7Days)
{
    const uint32_t SIMULATION_DURATION = 7 * 86400;
    const uint32_t TIME_STEP = 60;
    turtle.current_state = TurtleBehaviorModel::State::SURFACE;
    turtle.state_duration = turtle.generate_surface_duration();

    while (results.total_time_seconds < SIMULATION_DURATION) simulate_timestep(TIME_STEP);

    CHECK(results.dive_count >= 50);
    CHECK(results.max_depth > 20.0);
    CHECK(results.gps_fix_count >= 50);
    CHECK(results.argos_tx_count >= 10);

    printf("\n=== Simulation 7 jours ===\n");
    printf("Duree totale: %u secondes (%u jours)\n", results.total_time_seconds, results.total_time_seconds / 86400);
    printf("Plongees: %u, Remontees: %u\n", results.dive_count, results.surface_count);
    printf("Profondeur max: %.1f m\n", results.max_depth);
    printf("Temps surface: %.1f h (%.1f%%)\n", results.total_surface_time / 3600.0, 100.0 * results.total_surface_time / results.total_time_seconds);
    printf("Temps sous l'eau: %.1f h (%.1f%%)\n", results.total_underwater_time / 3600.0, 100.0 * results.total_underwater_time / results.total_time_seconds);
    printf("\n--- GPS ---\n");
    printf("Fixes reussis: %u\n", results.gps_fix_count);
    printf("TTF moyen: %.1f s\n", results.avg_ttf);
    printf("\n--- Capteurs ---\n");
    printf("Salinite: %.1f - %.1f PSU\n", results.min_salinity, results.max_salinity);
    printf("Temperature: %.1f - %.1f C\n", results.min_temp, results.max_temp);
    printf("Pression: %.1f - %.1f bar\n", results.min_pressure, results.max_pressure);
    printf("pH: %.2f - %.2f\n", results.min_ph, results.max_ph);
    printf("Luminosite: %.0f - %.0f lux\n", results.min_light, results.max_light);
    printf("\n--- Argos ---\n");
    printf("Transmissions: %u\n", results.argos_tx_count);
    printf("\n--- SWS ---\n");
    printf("Transitions detectees: %u\n", results.sws_transitions);
}

TEST(TurtleSimulation, SWSWithSalinityVariations)
{
    const uint32_t SIMULATION_DURATION = 6 * 3600;
    const uint32_t TIME_STEP = 1;
    std::vector<std::pair<uint32_t, uint16_t>> sws_readings;
    turtle.current_state = TurtleBehaviorModel::State::SURFACE;
    turtle.state_duration = 60;

    while (results.total_time_seconds < SIMULATION_DURATION) {
        simulate_timestep(TIME_STEP);
        if (results.total_time_seconds % 10 == 0) {
            uint16_t adc = environment.calculate_sws_adc(environment.current_salinity, turtle.current_depth, !turtle.is_at_surface());
            sws_readings.push_back({results.total_time_seconds, adc});
        }
        if (results.total_time_seconds % 3600 == 0 && results.total_time_seconds > 0) {
            environment.base_salinity = 10.0 + (double)(rand() % 25);
        }
    }

    uint16_t min_adc = 65535, max_adc = 0;
    double sum_adc = 0;
    for (const auto& reading : sws_readings) {
        if (reading.second < min_adc) min_adc = reading.second;
        if (reading.second > max_adc) max_adc = reading.second;
        sum_adc += reading.second;
    }

    printf("\n=== Test SWS avec variations salinite ===\n");
    printf("Lectures SWS: %zu\n", sws_readings.size());
    printf("ADC min: %u, max: %u, moyenne: %.0f\n", min_adc, max_adc, sum_adc / sws_readings.size());
    printf("Plage salinite: %.1f - %.1f PSU\n", results.min_salinity, results.max_salinity);
    printf("Transitions SWS: %u\n", results.sws_transitions);

    CHECK(max_adc > min_adc + 500);
    CHECK(results.sws_transitions > 10);
}

TEST(TurtleSimulation, GPSTTFVariousConditions)
{
    std::vector<uint32_t> hot_ttfs, warm_ttfs, cold_ttfs;
    const uint32_t SIMULATION_DURATION = 12 * 3600;
    const uint32_t TIME_STEP = 1;
    turtle.current_state = TurtleBehaviorModel::State::SURFACE;
    turtle.state_duration = 300;

    while (results.total_time_seconds < SIMULATION_DURATION) {
        bool had_fix = gps.has_fix;
        GPSModel::FixState state_before = gps.fix_state;
        simulate_timestep(TIME_STEP);
        if (gps.has_fix && !had_fix) {
            uint32_t ttf = gps.fix_attempt_time;
            switch (state_before) {
                case GPSModel::FixState::HOT: hot_ttfs.push_back(ttf); break;
                case GPSModel::FixState::WARM: warm_ttfs.push_back(ttf); break;
                case GPSModel::FixState::COLD:
                default: cold_ttfs.push_back(ttf); break;
            }
        }
    }

    auto avg = [](const std::vector<uint32_t>& v) -> double {
        if (v.empty()) return 0;
        double sum = 0;
        for (auto x : v) sum += x;
        return sum / v.size();
    };

    printf("\n=== Test GPS TTF ===\n");
    printf("Hot starts: %zu, TTF moyen: %.1f s\n", hot_ttfs.size(), avg(hot_ttfs));
    printf("Warm starts: %zu, TTF moyen: %.1f s\n", warm_ttfs.size(), avg(warm_ttfs));
    printf("Cold starts: %zu, TTF moyen: %.1f s\n", cold_ttfs.size(), avg(cold_ttfs));
    printf("Total fixes: %u\n", results.gps_fix_count);

    if (!hot_ttfs.empty() && !cold_ttfs.empty()) {
        CHECK(avg(hot_ttfs) < avg(cold_ttfs));
    }
}

TEST(TurtleSimulation, DeepDiveSensorReadings)
{
    const uint32_t SIMULATION_DURATION = 3600;
    const uint32_t TIME_STEP = 1;
    turtle.current_state = TurtleBehaviorModel::State::DIVING;
    turtle.target_depth = 200.0;
    turtle.state_duration = 3600;
    turtle.vertical_speed = 0.5;

    while (results.total_time_seconds < SIMULATION_DURATION) simulate_timestep(TIME_STEP);

    printf("\n=== Test plongee profonde ===\n");
    printf("Profondeur max atteinte: %.1f m\n", results.max_depth);
    printf("Temperature: %.1f - %.1f C\n", results.min_temp, results.max_temp);
    printf("Pression: %.1f - %.1f bar\n", results.min_pressure, results.max_pressure);
    printf("pH: %.2f - %.2f\n", results.min_ph, results.max_ph);

    CHECK(results.max_depth > 100.0);
    CHECK(results.min_temp < results.max_temp);
    CHECK(results.max_pressure > 10.0);
}
