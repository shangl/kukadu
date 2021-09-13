#ifndef KUKADU_KUKADUTYPES_H
#define KUKADU_KUKADUTYPES_H

    #include <exception>

    // shared_ptr
    #ifdef USEBOOST
        #include <boost/shared_ptr.hpp>
        #define KUKADU_SHARED_PTR boost::shared_ptr
        #define KUKADU_STATIC_POINTER_CAST boost::static_pointer_cast
        #define KUKADU_DYNAMIC_POINTER_CAST boost::dynamic_pointer_cast
        #define KUKADU_ENABLE_SHARED_FROM_THIS boost::enable_shared_from_this
    #else
        #include <memory>
        #define KUKADU_SHARED_PTR std::shared_ptr
        #define KUKADU_STATIC_POINTER_CAST std::static_pointer_cast
        #define KUKADU_DYNAMIC_POINTER_CAST std::dynamic_pointer_cast
        #define KUKADU_ENABLE_SHARED_FROM_THIS std::enable_shared_from_this
    #endif

    // thread
    #ifdef USEBOOST
        #include <boost/thread.hpp>
        typedef boost::thread kukadu_thread;
    #else
        #include <thread>
        typedef std::thread kukadu_thread;
    #endif

    // mutex
    #ifdef USEBOOST
        #include <boost/thread/mutex.hpp>
        typedef boost::mutex kukadu_mutex;
    #else
        #include <mutex>
        typedef std::mutex kukadu_mutex;
    #endif

    // random
    #ifdef USEBOOST
        #include <boost/random.hpp>
        #include <boost/random/mersenne_twister.hpp>
        #include <boost/random/normal_distribution.hpp>
        #include <boost/random/discrete_distribution.hpp>
        #include <boost/random/uniform_int_distribution.hpp>
        #include <boost/random/uniform_real_distribution.hpp>
        #define KUKADU_DISCRETE_DISTRIBUTION boost::random::discrete_distribution
        typedef boost::random::mt19937 kukadu_mersenne_twister;
        typedef boost::random::normal_distribution<double> kukadu_normal_distribution;
        typedef boost::random::uniform_int_distribution<int> kukadu_uniform_distribution;
        typedef boost::random::uniform_real_distribution<double> kukadu_uniform_real_distribution;
    #else
        #include <random>
        #define KUKADU_DISCRETE_DISTRIBUTION std::discrete_distribution
        typedef std::mt19937 kukadu_mersenne_twister;
        typedef std::uniform_int_distribution<int> kukadu_uniform_distribution;
        typedef std::normal_distribution<double> kukadu_normal_distribution;
        typedef std::uniform_real_distribution<double> kukadu_uniform_real_distribution;
    #endif

namespace kukadu {

    class KukaduException : public std::exception {

    private:

        const char* message;

    public:

        KukaduException(const char* message) { this->message = message; }

        virtual const char* what() const throw() {
            return message;
        }
    };

    class TimedObject {

    public:

        /**
         * \brief An intrinsic clock is started with startQueue(). This method returns
         * the current time according to this clock.
         * \return current time
         */
        static long long int getCurrentTime();

    };

}

#endif
