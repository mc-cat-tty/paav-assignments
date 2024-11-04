#include <iostream>
#include <string>


namespace logger {
    /**
    @brief Singleton compliant to the builder pattern
     */
    class Logger {
        public:
            Logger(const std::string &tag) : tag(tag) {}

            template<typename T>
            std::ostream& operator<<(T out_data) {
                std::cerr << "[" << tag << "]: " << out_data;
                return std::cerr;                    
            }

        
        private:
            std::string tag;
    };
}