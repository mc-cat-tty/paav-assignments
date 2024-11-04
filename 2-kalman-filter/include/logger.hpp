#include <iostream>
#include <string>


namespace logger {
    /**
    @brief Singleton compliant to the builder pattern
     */
    class Logger {
        public:
            static Logger& getInstance() {
                static Logger logger;
                return logger;
            }

            void logDistance(
                unsigned tracklet_id, unsigned subject_id,
                double euclidean_distance, double mahalanobis_distance,
                std::string tag = "DISTANCE"
            ) {
                std::cerr << "[" << tag << "] "
                    << tracklet_id << " - " << subject_id << ": "
                    << euclidean_distance << " m (euclidean) vs " << mahalanobis_distance << "m (mahalanobis)"
                    << std::endl;
            }
        
        private:
            Logger() = default;
            Logger(const Logger&) = delete;
            Logger(Logger&&) = delete;
            Logger& operator=(const Logger&) = delete;
    };
}