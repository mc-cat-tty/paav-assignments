#pragma once

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
                #if (LOGGING_ON)
                std::cerr << "[" << tag << "]: " << out_data << std::endl;
                #endif
                return std::cerr;                    
            }

        
        private:
            std::string tag;
    };
}