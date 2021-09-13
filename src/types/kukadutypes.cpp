#include <chrono>
#include <kukadu/types/kukadutypes.hpp>

using namespace std;
using namespace std::chrono;

namespace kukadu {

    long long int TimedObject::getCurrentTime() {
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }

}
