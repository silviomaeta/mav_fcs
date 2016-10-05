/*
 * fcs_state_machine.h
 * Author: Silvio Maeta
 */

#ifndef _MAV_FCS_STATE_MACHINE_H_
#define _MAV_FCS_STATE_MACHINE_H_

#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <ros/ros.h>

namespace mpl = boost::mpl;

namespace sc = boost::statechart;

namespace mavfcs {

    enum FcsState {
        SM_DEFAULT = 0,
        SM_ONGROUND,
        SM_TAKINGOFF,
        SM_EXECUTINGCOMMAND,
        SM_LANDING,
        SM_HOVER,
        SM_DISENGAGED,
        SM_INVALID
    };

    struct InputFSM {
        //When system is initiated: onground / hover
        FcsState initialState;

        //Current FCS state machine internal state
        FcsState currentState;
        //Keep time state changed - some transitions triggered by time
        double stateChangeTime;
        
        bool isEngaged;
        bool isOnGround;
        
        bool hasTakingOffCommand;
        bool takeOffComplete;
        
        bool hasLandingCommand;
        bool landingComplete;
        
        bool hasTrajectoryCommand;
        bool trajectoryComplete;

        //Flag that tells if we are running a simulation or the real system
        //bool isSimulation;
        
        //Received externally
        //bool hasCurrentPose;
        //nav_msgs::Odometry currentPose;
                        
        //Variables to control take-off 
        //bool hasStartPose;
        //nav_msgs::Odometry startPose;
    };


    struct FlyingStateMachine;

    class FcsStateMachine {
    
    private:
        FlyingStateMachine *m_fsm;

    public:
        FcsStateMachine(InputFSM *input);

        ~FcsStateMachine();

        int getCurrentState();

        void update();
        
        void engage(void);
        void disengage(void);
        
        void onGround(void);
        void inAir(void);
        
        void takeOffCommanded(void);
        void takeOffCompleted(void);
        
        void landingCommanded(void);
        void landingCompleted(void);
        
        void trajectoryCommanded(void);
        void trajectoryCompleted(void);
    };


}


#endif /* _MAV_FCS_STATE_MACHINE_H_ */
