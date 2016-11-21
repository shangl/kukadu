#ifndef KUKADU_KINESTHETICTEACHER_HPP
#define KUKADU_KINESTHETICTEACHER_HPP

namespace kukadu {

    class KinestheticTeacher {

    public:

        virtual void init() = 0;
        virtual void startTeaching() = 0;
        virtual void stopTeaching() = 0;
        virtual void startRecording() = 0;
        virtual void stopRecording() = 0;
        virtual void quit() = 0;

    };

}

#endif
