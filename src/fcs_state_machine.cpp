/*
 * fcs_state_machine.cpp
 * Author: Silvio Maeta
 */

#include "mav_fcs/fcs_state_machine.h"

using namespace mavfcs;

namespace mavfcs {

    struct EvTransition: sc::event< EvTransition > {
      EvTransition() {}
    };

    struct EvReset: sc::event< EvReset > {
      EvReset() {}
    };

    struct ActiveProblem;

    struct FlyingStateMachine : sc::state_machine< FlyingStateMachine, ActiveProblem > {
        InputFSM *input;
    };

    struct DefaultState;
    struct DisengagedState;
    struct TakingOffState;
    struct HoverState;
    struct ExecutingCommandState;
    struct LandingState;
    struct OnGroundState;

    struct ActiveProblem : sc::simple_state<ActiveProblem, FlyingStateMachine, DefaultState > {
      typedef mpl::list< sc::transition<EvReset, DefaultState>, sc::transition<EvTransition,DefaultState > > reactions;
      ActiveProblem(){ROS_INFO_STREAM("--> FCS ActiveProblem\n");}
      ~ActiveProblem() {ROS_INFO_STREAM("<-- FCS ActiveProblem\n");}

    };

    //--------------------------------------------------------------------------

    struct DefaultState :  sc::state< DefaultState, ActiveProblem > {

      typedef mpl::list< sc::transition< EvReset, DefaultState> , sc::custom_reaction< EvTransition > > reactions;

      DefaultState(my_context ctx): my_base(ctx) {
        ROS_INFO_STREAM("--> FCS Default State \n");

        InputFSM *input = context< FlyingStateMachine >().input;
        input->currentState = input->initialState;
        input->stateChangeTime = ros::Time().now().toSec();
      }

      ~DefaultState() {ROS_INFO_STREAM("<-- FCS Default State \n");}

      sc::result react(const EvTransition &ev) {
        InputFSM *input = context< FlyingStateMachine >().input;
     
        //Only goes to the next state if received pose information    
        //if (input->hasCurrentPose) {
        
          if (input->initialState == SM_ONGROUND) {
            return transit <OnGroundState >();
          }
          else if (input->initialState == SM_HOVER) {
            return transit <HoverState >();
          }
        
        //}
        if (! input->isEngaged) {
          return transit <DisengagedState >();
        }
     
        return discard_event();
      }

    };

    //--------------------------------------------------------------------------

    struct DisengagedState :  sc::state< DisengagedState, ActiveProblem > {
      
      typedef mpl::list< sc::transition< EvReset, DisengagedState> , sc::custom_reaction< EvTransition > > reactions;
      
      DisengagedState(my_context ctx): my_base(ctx) {
        ROS_INFO_STREAM("--> FCS Disengaged State \n");

        InputFSM *input = context< FlyingStateMachine >().input;
        input->currentState = SM_DISENGAGED;
        input->stateChangeTime = ros::Time().now().toSec();
      }

      ~DisengagedState() {ROS_INFO_STREAM("<-- FCS Disengaged State \n");}

      sc::result react(const EvTransition &ev)
      {
        InputFSM *input = context< FlyingStateMachine >().input;
        
        if (input->isEngaged) { 
          if (input->isOnGround) {
            return transit <OnGroundState >();
          }
          else {        
            return transit <HoverState >();
          }
        }
   
        return discard_event();
      }

    };

    //--------------------------------------------------------------------------

    struct TakingOffState :  sc::state< TakingOffState, ActiveProblem > {
      
      typedef mpl::list< sc::transition< EvReset, TakingOffState> , sc::custom_reaction< EvTransition > > reactions;
      
      TakingOffState(my_context ctx): my_base(ctx) {
        ROS_INFO_STREAM("--> FCS Takeoff State \n");

        InputFSM *input = context< FlyingStateMachine >().input;
        input->currentState = SM_TAKINGOFF;
        input->stateChangeTime = ros::Time().now().toSec();
      }

      ~TakingOffState() {ROS_INFO_STREAM("<-- FCS Takeoff State \n");}

      sc::result react(const EvTransition &ev) {
            
        InputFSM *input = context< FlyingStateMachine >().input;
        
        if (input->takeOffComplete) {
          if (input->hasTrajectoryCommand) {
            return transit <ExecutingCommandState >();            
          }
          else {
            return transit <HoverState >();            
          }
        }
        if (! input->isEngaged) {
          return transit <DisengagedState >();
        }

        return discard_event();
      }

    };

    //--------------------------------------------------------------------------

    struct HoverState :  sc::state< HoverState, ActiveProblem > {
    
      typedef mpl::list< sc::transition< EvReset, HoverState> , sc::custom_reaction< EvTransition > > reactions;
      
      HoverState(my_context ctx): my_base(ctx) {
        ROS_INFO_STREAM("--> FCS Hover State \n");

        InputFSM *input = context< FlyingStateMachine >().input;
        input->currentState = SM_HOVER;
        input->stateChangeTime = ros::Time().now().toSec();
      }

      ~HoverState() {ROS_INFO_STREAM("<-- FCS Hover State \n");}

      sc::result react(const EvTransition &ev) {
        InputFSM *input = context< FlyingStateMachine >().input;

        if (input->hasTrajectoryCommand) {
          return transit <ExecutingCommandState >();
        }
        else if (input->hasLandingCommand) {
          return transit <LandingState >();
        }
        if (! input->isEngaged) {
          return transit <DisengagedState >();
        }
        
        return discard_event();
      }

    };

    //--------------------------------------------------------------------------
    
    struct ExecutingCommandState :  sc::state< ExecutingCommandState, ActiveProblem > {

      typedef mpl::list< sc::transition< EvReset, ExecutingCommandState> , sc::custom_reaction< EvTransition > > reactions;

      ExecutingCommandState(my_context ctx): my_base(ctx) {
        ROS_INFO_STREAM("--> FCS ExecutingCommand State \n");
        InputFSM *input = context< FlyingStateMachine >().input;
        input->currentState = SM_EXECUTINGCOMMAND;
        input->stateChangeTime = ros::Time().now().toSec();
      }

      ~ExecutingCommandState() {ROS_INFO_STREAM("<-- FCS ExecutingCommand State \n");}

      sc::result react(const EvTransition &ev) {
        InputFSM *input = context< FlyingStateMachine >().input;

        if (input->trajectoryComplete) {
          if (input->hasLandingCommand) {
            return transit <LandingState >();
          }
          else {
            return transit <HoverState >();          
          }
        }
        if (! input->isEngaged) {
          return transit <DisengagedState >();
        }
        
        return discard_event();
      }

    };

    //--------------------------------------------------------------------------
    
    struct LandingState :  sc::state< LandingState, ActiveProblem > {
    
      typedef mpl::list< sc::transition< EvReset, OnGroundState> , sc::custom_reaction< EvTransition > > reactions;

      LandingState(my_context ctx): my_base(ctx) {
        ROS_INFO_STREAM("--> FCS Landing State \n");

        InputFSM *input = context< FlyingStateMachine >().input;
        input->currentState = SM_LANDING;
        input->stateChangeTime = ros::Time().now().toSec();
      }

      ~LandingState() {ROS_INFO_STREAM("<-- FCS Landing State \n");}

      sc::result react(const EvTransition &ev) {      
        InputFSM *input = context< FlyingStateMachine >().input;

        if (input->landingComplete) {        
          return transit <OnGroundState >();                
        }
        if (! input->isEngaged) {
          return transit <DisengagedState >();
        }
        
        return discard_event();
      }

    };
    
    //--------------------------------------------------------------------------
    
    struct OnGroundState :  sc::state< OnGroundState, ActiveProblem > {

      typedef mpl::list< sc::transition< EvReset, OnGroundState> , sc::custom_reaction< EvTransition > > reactions;

      OnGroundState(my_context ctx): my_base(ctx) {
        ROS_INFO_STREAM("--> FCS OnGround State \n");

        InputFSM *input = context< FlyingStateMachine >().input;
        input->currentState = SM_ONGROUND;
        input->stateChangeTime = ros::Time().now().toSec();
      }

      ~OnGroundState() {ROS_INFO_STREAM("<-- FCS OnGround State \n");}

      sc::result react(const EvTransition &ev) {
      
        InputFSM *input = context< FlyingStateMachine >().input;
        
        if (input->hasTakingOffCommand) {
          return transit <TakingOffState >();
        }
        if (! input->isEngaged) {
          return transit <DisengagedState >();
        }
                
        return discard_event();
      }

    };

}

//==============================================================================

FcsStateMachine::FcsStateMachine(struct InputFSM *inputdata):
  m_fsm(new FlyingStateMachine()) {
  m_fsm->input = inputdata;
  
  inputdata->initialState = SM_ONGROUND;
  inputdata->isEngaged = true;
  inputdata->isOnGround = true;
  inputdata->hasTakingOffCommand = false;
  inputdata->takeOffComplete = false;
  inputdata->hasLandingCommand = false;
  inputdata->landingComplete = false;
  inputdata->hasTrajectoryCommand = false;
  inputdata->trajectoryComplete = false;
  
  m_fsm->initiate();
}

FcsStateMachine::~FcsStateMachine() {
    delete m_fsm;
}

int FcsStateMachine::getCurrentState() {
    return m_fsm->input->currentState;
}

void FcsStateMachine::update() {
    ROS_INFO_STREAM_THROTTLE(5.0, "[FCS Sim] State machine update");
    m_fsm->process_event( EvTransition() );
}

void FcsStateMachine::engage(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->isEngaged = true;
}

void FcsStateMachine::disengage(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->isEngaged = false;
}

void FcsStateMachine::onGround(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->isOnGround = true;
}

void FcsStateMachine::inAir(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->isOnGround = false;
}

void FcsStateMachine::takeOffCommanded(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->hasTakingOffCommand = true;
    ifsm->takeOffComplete = false;    
}

void FcsStateMachine::takeOffCompleted(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->hasTakingOffCommand = false;
    ifsm->takeOffComplete = true;
}
 
void FcsStateMachine::landingCommanded(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->hasLandingCommand = true;
    ifsm->landingComplete = false;    
}

void FcsStateMachine::landingCompleted(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->hasLandingCommand = false;
    ifsm->landingComplete = true;
}
        
void FcsStateMachine::trajectoryCommanded(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->hasTrajectoryCommand = true;
    ifsm->trajectoryComplete = false;
}

void FcsStateMachine::trajectoryCompleted(void) {
    InputFSM *ifsm = m_fsm->input;
    ifsm->hasTrajectoryCommand = false;
    ifsm->trajectoryComplete = true;
}

