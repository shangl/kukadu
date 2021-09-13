#ifndef KUKADU_DESTROYABLEOBJECT
#define KUKADU_DESTROYABLEOBJECT

namespace kukadu {

    /**
     * \brief Interface defining a callback function
     *
     * The specified callback function should be called whenever unexpected behaviour during robot execution occurs. This is used to handle this execption in a controlled way.
     */
    class DestroyableObject {

    private:

    public:

        /** \brief Method is called whenever destroy event occurs and ensures safe and clean termination of the programm (e.g. stops robot)
         *
         */
        virtual void safelyDestroy() = 0;

    };

}

#endif
