#include <boost/format.hpp>
#include <Motion/motion.h>
#include <Optim/optimization.h>
#include <kukadu/utils/utils.hpp>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_default.h>
#include <kukadu/planning/komo.hpp>
#include <Motion/taskMap_transition.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_constrained.h>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;
using namespace ros;
using namespace arma;

namespace kukadu {

    struct PlanningResult {

        int status;

        double planning_time;

        std::string error_msg;

        ors::Vector pos_error;
        ors::Vector ang_error;

        ors::Transformation resulting_pose;
        trajectory_msgs::JointTrajectory path;

    };

    kukadu_mutex Komo::oneAtATimeMutex;


    /****************** public functions *******************************/

    Komo::Komo(KUKADU_SHARED_PTR<ControlQueue> queue, string configPath, string mtConfigPath, vector<std::string>& activeJointsPrefixes, bool acceptCollision)
                : PathPlanner(generateDefaultJointNames(queue->getDegreesOfFreedom())) {

        oneAtATimeMutex.lock();

            auto generatedJointNames = generateDefaultJointNames(queue->getDegreesOfFreedom());

            this->queue = queue;
            this->activeJointsPrefixes = activeJointsPrefixes;
            MT::openConfigFile(mtConfigPath.c_str());

            simplePlanner = make_shared<SimplePlanner>(queue, KUKADU_SHARED_PTR<Kinematics>(), generatedJointNames);

            _world = new ors::KinematicWorld(configPath.c_str());
            _world->swift().initActivations(*_world);

            // initialize list of activated joints
            for(ors::Joint* j : _world->joints) {

                if(j->agent == 0) {

                    string currJointName = string((char *) j->name);

                    for (auto activeJointsPrefix : activeJointsPrefixes) {
                        if (currJointName.find(activeJointsPrefix) != string::npos) {
                            _active_joints.push_back(j);
                            sJointNames.push_back(currJointName);
                            break;
                        }
                    }

                }

            }

            double def_pos_tolerance = MT::getParameter<double>("KOMO/moveTo/defaultPositionTolerance", 0.005);
            double def_ang_tolerance = MT::getParameter<double>("KOMO/moveTo/defaultAngularTolerance", 0.1);

            _pos_tolerance = {def_pos_tolerance, def_pos_tolerance, def_pos_tolerance};
            _ang_tolerance = {def_ang_tolerance, def_ang_tolerance, def_ang_tolerance};

            _positionPrecision = MT::getParameter<double>("KOMO/moveTo/positionPrecision", 1e4); // original 1e3
            _collisionPrecision = MT::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
            _collisionMargin = MT::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
            _jointLimitPrecision = MT::getParameter<double>("KOMO/moveTo/jointLimitPrecision", 0.1);
            _jointLimitMargin = MT::getParameter<double>("KOMO/moveTo/jointLimitMargin", 1e5);
            _jointStatePrecision = MT::getParameter<double>("KOMO/moveTo/jointStatePrecision", 1e5);
            _zeroVelocityPrecision = MT::getParameter<double>("KOMO/moveTo/zeroVelocityPrecision", 1e1);
            _alignmentPrecision = MT::getParameter<double>("KOMO/moveTo/alignmentPrecision", 1e4); // original 1e3
            _maxIterations = MT::getParameter<double>("KOMO/moveTo/maxIterations", 1);

            _support_surface_name = MT::getParameter<MT::String>("KOMO/scene/supportSurfaceName");
            _world_link_name = MT::getParameter<MT::String>("KOMO/scene/worldLinkName");

            eef_link = activeJointsPrefixes.front() + string("_sdh_palm_link");
            allowContact(eef_link.c_str(), false);

            this->acceptCollision = acceptCollision;

        oneAtATimeMutex.unlock();

    }

    void Komo::setSpeed(double speed) {
        simplePlanner->setSpeed(speed);
    }

    Komo::~Komo() {

        if(_world)
            delete _world;

    }

    std::vector<arma::vec> Komo::planJointTrajectory(std::vector<arma::vec> intermediateJoints) {

        KUKADU_MODULE_START_USAGE();

        std::vector<arma::vec> retTrajectory;
        PlanningResult result;

        setState(sJointNames, queue->getCurrentJoints().joints);

        std::vector<double> goal_state = armadilloToStdVec(intermediateJoints.back());
        if(goal_state.size() != _active_joints.size())
            throw KukaduException("(Komo)  size of joint vector does not fit the number of joint names");

        MT::timerStart();

        arr state = _world->getJointState();

        // initialize goal configuration with current state and then assign desired goal values
        arr goal_config = state;
        for(int i = 0; i < _active_joints.size(); ++i) {
            int idx = _active_joints[i]->qIndex;
            goal_config(idx) = goal_state[i];
        }

        // create the motion problem
        MotionProblem MP(*_world);
        TaskCost* c;

        // TaskMap for goal state
        c = MP.addTask("state", new DefaultTaskMap(qItselfTMT, *_world));
        c->setCostSpecs(MP.T - 10, MP.T, goal_config, _jointStatePrecision);

        // TaskMap for zero velocity at goal
        c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, *_world));
        c->setCostSpecs(MP.T, MP.T, {0.}, _zeroVelocityPrecision);
        c->map.order = 1; //make this a velocity variable!

        // Constraint to enforce joint limits on all time slices
        LimitsConstraint *lc = new LimitsConstraint();
        lc->margin = 0.005;
        c = MP.addTask("Joint_limits", lc);
        c->setCostSpecs(0, MP.T, {0.}, _jointLimitPrecision);

        // enable collision checking
        TaskCost *colCost = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, _collisionMargin));
        colCost->setCostSpecs(0, MP.T, {0.}, _collisionPrecision);

        // TaskMap for transition costs
        c = MP.addTask("Transitions", new TransitionTaskMap(*_world));
        c->map.order = 2; // penalize accelerations
        c->setCostSpecs(0, MP.T, {0.}, 1e0);

        //-- create the Optimization problem (of type kOrderMarkov)
        MotionProblemFunction MF(MP);
        ors::KinematicWorld::setJointStateCount = 0;
        arr traj;
        bool goal_state_reached = false;

        for(int j = 0; j < 10; ++j) {

            MP.prefix.clear();
            MP.x0 = state;

            arr x;
            // initialize as interpolation from current state to desired goal state
            sineProfile(x, state, goal_config, MP.T);
            // and then do the optimization
            optConstrained(x, NoArr, Convert(MF), OPT(verbose = 0, stopIters = 100, maxStep = .5, stepInc = 2., allowOverstep = false));
            //MP.costReport(false);

            // ensure that all joints are within calculated limits before do collision validation
            ensureJointLimits(*_world, x);

            if(!validateCollisions(*_world, x, result.error_msg)) {
                result.status = RESULT_FAILED;
                cerr << "(Komo) Collision Validation failed!" << endl;
                if(!acceptCollision)
                    return retTrajectory;
            }
            // not necessary any more - just for test purposes...
            if(!validateJointLimits(*_world, x, result.error_msg)) {
                result.status = RESULT_FAILED;
                cerr << "(Komo) Joint Limit Validation failed!" << endl;
                return retTrajectory;
            }

            traj.append(x);

            state = x[MP.T - 1];

            if(check_goal_state(goal_config, state)) {
                goal_state_reached = true;
                break;
            }

        }

        CHECK(traj.d0 > 0, "Trajectory is empty...");

        result.planning_time = MT::timerRead();

        // consider plan to be successful unless validation shows
        // something different...
        result.status = RESULT_SUCCESS;

        // so far, our path is valid but maybe the goal tolerances are violated...
        pathToTrajectory(result.path, traj);

        for(auto point : result.path.points)
            retTrajectory.push_back(stdToArmadilloVec(point.positions));

        // clear the proxies to clean up the UI
        listDelete(_world->proxies);

        KUKADU_MODULE_END_USAGE();

        return simplePlanner->planJointTrajectory(retTrajectory);

    }

    std::vector<arma::vec> Komo::planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        KUKADU_MODULE_START_USAGE();

        vector<arma::vec> finalJointPlan;

        if(useCurrentRobotState)
            finalJointPlan = planJointTrajectory({queue->getCurrentJoints().joints, startJoints});

        if(!intermediatePoses.empty()) {

            auto lastEndJoint = startJoints;

            for(auto nextPose : intermediatePoses) {

                auto jointPlan = computeSinglePlan(lastEndJoint, {nextPose}, false, false);
                lastEndJoint = jointPlan.back();
                finalJointPlan.insert(finalJointPlan.end(), jointPlan.begin(), jointPlan.end());

            }

        }

        finalJointPlan = simplePlanner->planJointTrajectory(finalJointPlan);

        if(smoothCartesians)
            finalJointPlan = smoothJointPlan(finalJointPlan, {0.0}, queue->getCycleTime());

        KUKADU_MODULE_END_USAGE();

        return finalJointPlan;

    }

    std::vector<arma::vec> Komo::planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) {

        return planCartesianTrajectory(queue->getCurrentJoints().joints, intermediatePoses, smoothCartesians, useCurrentRobotState);

    }

    geometry_msgs::Pose Komo::computeFk(arma::vec joints) {

        KUKADU_MODULE_START_USAGE();

        geometry_msgs::Pose retPose;
        ors::Shape* endeff = _world->getShapeByName(eef_link.c_str());
        setState(sJointNames, joints);
        ors::Transformation trans = endeff->X;
        retPose.position.x = trans.pos(0); retPose.position.y = trans.pos(1); retPose.position.z = trans.pos(2);
        retPose.orientation.x = trans.rot.x;
        retPose.orientation.y = trans.rot.y;
        retPose.orientation.z = trans.rot.z;
        retPose.orientation.w = trans.rot.w;

        KUKADU_MODULE_END_USAGE();

        return retPose;

    }

    geometry_msgs::Pose Komo::computeFk(std::vector<double> jointState) {

        return computeFk(stdToArmadilloVec(jointState));

    }

    std::vector<arma::vec> Komo::computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) {
        throw KukaduException("(Komo) ik solver not supported yet");
    }

    bool Komo::isColliding(arma::vec jointState, geometry_msgs::Pose pose) {
        throw KukaduException("(Komo) isColliding not implemented yet");
    }

    Eigen::MatrixXd Komo::getJacobian(std::vector<double> jointState) {
        throw KukaduException("(Komo) getJacobian not implemented yet");
    }

    std::string Komo::getCartesianLinkName() {
        return eef_link;
    }

    std::string Komo::getCartesianReferenceFrame() {
        // hack for now: don't know how to find out the reference frame with komo (or if it even has that defined)
        return "origin";
    }

    /****************** private functions ******************************/

    void Komo::setJointPosition(const string &name, const double pos) {

        ors::Joint *jnt = _world->getJointByName(name.c_str());
        if(!jnt)
            cerr << "Unable to set joint position - no joint with name '" << name << "' found!" << endl;
        else
            jnt->Q.rot.setRad(pos, 1, 0, 0);

    }

    void Komo::setState(const sensor_msgs::JointState &state) {

        CHECK(state.name.size() == state.position.size(), "Illegal joint states!");
        for(size_t i = 0; i < state.name.size(); ++i) {
            setJointPosition(state.name[i], state.position[i]);
        }
        // update mimic joints
        for(ors::Joint *j:_world->joints) {
            if(j->mimic)
                // not very nice, but should work...
                j->Q.rot = j->mimic->Q.rot;
        }
        _world->calc_fwdPropagateFrames();
        _world->calc_q_from_Q();

    }

    void Komo::setState(const std::vector<std::string>& jointNames, const arma::vec& joints) {
        sensor_msgs::JointState state;
        for(size_t i = 0; i < joints.n_elem; ++i) {
            state.name.push_back(jointNames.at(i));
            state.position.push_back(joints(i));
        }
        setState(state);
    }

    std::vector<arma::vec> Komo::computeSinglePlan(arma::vec startJoints, geometry_msgs::Pose targetPose, bool smoothCartesians, bool useCurrentRobotState) {

        std::vector<arma::vec> retTrajectory;
        PlanningResult result;

        ors::Transformation goal;
        // geometry_msgs::Pose startPose = intermediatePoses.at(0);
        geometry_msgs::Pose endPose = targetPose;
        goal.pos.x = endPose.position.x; goal.pos.y = endPose.position.y; goal.pos.z = endPose.position.z;
        goal.rot.set(endPose.orientation.w, endPose.orientation.x, endPose.orientation.y, endPose.orientation.z);
        goal.rot.normalize();

        setState(sJointNames, startJoints);
        int axis_to_align = 7;

        // ensure that some joints have been enabled, otherwise planning is not possible
        CHECK(_active_joints.size() > 0, "Unable to plan - at least 1 joint has to be activated!");
        MT::timerStart();

        ors::Shape *target = _world->getShapeByName("target");
        if(!target) {
            result.error_msg = "Unable to find shape with name 'target' within model.";
            cerr << result.error_msg << endl;
            result.status = RESULT_FAILED;
            return retTrajectory;
        }

        ors::Shape* endeff = _world->getShapeByName(eef_link.c_str());
        if(!endeff) {
            result.error_msg = "Unable to find link with name '" + eef_link + "' within model.";
            cerr << result.error_msg << endl;
            result.status = RESULT_FAILED;
            return retTrajectory;
        }

        // set target position...
        target->rel.pos = goal.pos;
        // ...and orientation
        target->rel.rot = goal.rot;

        _world->calc_fwdPropagateShapeFrames();
        // display(false, "planning...");

        target->cont = false; // don't know if this is necessary...

        //-- set up the MotionProblem
        MotionProblem MP(*_world);
        MP.loadTransitionParameters();
        _world->swift().initActivations(*_world);

        TaskCost *c;
        // TaskMap for end effector position
        c = MP.addTask("EEF_position", new DefaultTaskMap(posTMT, endeff->index, NoVector, target->index, NoVector));
        c->setCostSpecs(MP.T, MP.T, {0.}, _positionPrecision);

        // TaskMap for zero velocity at goal
        c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, *_world));
        c->setCostSpecs(MP.T, MP.T, {0.}, _zeroVelocityPrecision);
        c->map.order = 1; //make this a velocity variable!

        // TaskMaps for eef alignment
        for(uint i=0; i < 3; i++) if(axis_to_align & (1 << i)) {
            ors::Vector axis;
            axis.setZero();
            axis(i) = 1.;
            c = MP.addTask(STRING("allign_" << i), new DefaultTaskMap(vecAlignTMT, endeff->index, axis, target->index, axis));
            c->setCostSpecs(MP.T, MP.T, {1.}, _alignmentPrecision); // ARR(0.) -> make axis orthogonal!
        }

        // Constraint to enforce joint limits on all time slices
        LimitsConstraint *lc = new LimitsConstraint();
        lc->margin = 0.005;
        c = MP.addTask("Joint_limits", lc);
        c->setCostSpecs(0, MP.T, {0.}, _jointLimitPrecision);

        // enable collision checking
        TaskCost* colCost = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, _collisionMargin));
        colCost->setCostSpecs(MP.T, MP.T, {0.}, _collisionPrecision);

        // TaskMap for transition costs
        c = MP.addTask("Transitions", new TransitionTaskMap(*_world));
        c->map.order = 2; // penalize accelerations
        c->setCostSpecs(0, MP.T, {0.}, 1e0);

        //-- create the Optimization problem (of type kOrderMarkov)
        MotionProblemFunction MF(MP);
        //-- optimization process

        ors::KinematicWorld::setJointStateCount = 0;
        arr state, traj;
        ors::Vector posError, angError;

        _world->getJointState(state);

        for(int j = 0; j < _maxIterations; ++j) {

            MP.prefix.clear();
            MP.x0 = state;

            arr x = replicate(MP.x0, MP.T + 1);
            rndGauss(x, .01, true); //don't initialize at a singular config

            optConstrained(x, NoArr, Convert(MF), OPT(verbose = 0, stopIters = 100, maxStep = .5, stepInc = 2., allowOverstep = false));
            // MP.costReport(false);

            // ensure that all joints are within calculated limits before doing collision validation
            ensureJointLimits(*_world, x);

            if(!validateCollisions(*_world, x, result.error_msg)) {
                result.status = RESULT_FAILED;
                cerr << "(Komo) Collision Validation failed!" << endl;
                if(!acceptCollision)
                    return retTrajectory;
            }
            // not necessary any more - just for test purposes...
            if(!validateJointLimits(*_world, x, result.error_msg)) {
                result.status = RESULT_FAILED;
                cerr << "(Komo) Joint Limit Validation failed!" << endl;
                return retTrajectory;
            }

            traj.append(x);

            // set world to final state of resulting path and calculate the transformations
            state = x[x.d0 - 1];
            _world->setJointState(state);

            computePositionError(*endeff, *target, posError);
            computeAlignmentError(*endeff, *target, angError, axis_to_align);

            if(withinTolerance(posError, _pos_tolerance) &&
               withinTolerance(angError, _ang_tolerance, axis_to_align))
            {
                break;
            }
        }

        CHECK(traj.d0 > 0, "Trajectory is empty...");

        result.planning_time = MT::timerRead();

        /* ---------------------- validation -------------------------*/

        // consider plan to be successful unless validation shows
        // something different...
        result.status = RESULT_SUCCESS;

        // so far, our path is valid but maybe the goal tolerances are violated...
        pathToTrajectory(result.path, traj);
        result.resulting_pose = endeff->X;
        result.pos_error = posError;
        result.ang_error = angError;

        // check end effector goal position
        if(!withinTolerance(result.pos_error, _pos_tolerance)) {
            result.error_msg.append("Goal position not within tolerance values! ");
            result.status = RESULT_APPROXIMATE;
        }

        // check end effector goal alignment
        if(!withinTolerance(result.ang_error, _ang_tolerance, axis_to_align)) {
            result.error_msg.append("Goal alignment not within tolerance values! ");
            result.status = RESULT_APPROXIMATE;
        }

        for(auto point : result.path.points)
            retTrajectory.push_back(stdToArmadilloVec(point.positions));

        // clear the proxies to clean up the UI
        listDelete(_world->proxies);

        return simplePlanner->planJointTrajectory(retTrajectory);

    }

    /*
     *
     * helper functions
     *
     */
    double Komo::keyframeOptimizer(arr& x, MotionProblem& MP, bool x_is_initialized, uint verbose) {

        MotionProblem_EndPoseFunction MF(MP);

        if (!x_is_initialized) x=MP.x0;

        double cost;

        optNewton(x, Convert(MF), OPT(fmin_return=&cost, verbose=verbose, stopIters=200, damping=1e-0, maxStep=.5, stopTolerance=1e-2));

        return cost;
    }

    double Komo::optimizeEndpose(arr &xT, ors::KinematicWorld &w, const char *link, const char *target, bool allowCollision) {

        double posPrec = 1e4;
        double alignPrec = 1e5; // original 1e3
        double collPrec = 1e1;
        double collMarg = 0.02;
        double jntLimitPrec = 1e1;

        //-- set up the MotionProblem
        arr hrate(w.getJointStateDimension());
        for (int i = 0; i < hrate.N; ++i) {
            hrate(i) = 0.0;
        }

        MotionProblem MP(w);
        MP.H_rate_diag = hrate;

        TaskCost *c;

        // TaskMap for end effector position
        c = MP.addTask("position", new DefaultTaskMap(posTMT, w, link, NoVector, target, NoVector));
        c->setCostSpecs(MP.T, MP.T, {0.}, posPrec);

        c = MP.addTask("alignX", new DefaultTaskMap(vecAlignTMT, w, link, {1,0,0}, target, {1,0,0}));
        c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);

        c = MP.addTask("alignY", new DefaultTaskMap(vecAlignTMT, w, link, {0,1,0}, target, {0,1,0}));
        c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);

        c = MP.addTask("alignZ", new DefaultTaskMap(vecAlignTMT, w, link, {0,0,1}, target, {0,0,1}));
        c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);

        // TaskMap to enforce joint limits on all time slices
        c = MP.addTask("Joint_limits", new DefaultTaskMap(qLimitsTMT));
        c->setCostSpecs(0, MP.T, {0.}, jntLimitPrec);

        if(!allowCollision) {
            // enable collision checking
            c = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, collMarg));
            c->setCostSpecs(0, MP.T, {0.}, collPrec);
        }

        return keyframeOptimizer(xT, MP, false, 1);
    }

    void Komo::computePositionError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error) {
        CHECK(&eef && &target, "One of the provided shapes does not exist!");
        error = eef.X.pos - target.X.pos;
    }

    void Komo::computePositionError(ors::KinematicWorld &w, const char *eef, const char *target, ors::Vector &error) {
        ors::Shape *_eef = w.getShapeByName(eef);
        ors::Shape *_target = w.getShapeByName(target);
        return computePositionError(*_eef, *_target, error);
    }

    void Komo::computeAlignmentError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error, int axes) {
        CHECK(&eef && &target, "One of the provided shapes does not exist!");

        ors::Vector tx,ty,tz, ex, ey, ez;

        target.X.rot.getX(tx);
        target.X.rot.getY(ty);
        target.X.rot.getZ(tz);
        eef.X.rot.getX(ex);
        eef.X.rot.getY(ey);
        eef.X.rot.getZ(ez);
        error.x = tx.angle(ex);
        error.y = ty.angle(ey);
        error.z = tz.angle(ez);

        // clear error value on axes we do not have to consider
        for(int i = 0; i < 3; ++i) if(!(axes & (1 << i))) {
            error(i) = 0.;
        }
    }

    void Komo::computeAlignmentError(ors::KinematicWorld &w, const char *eef, const char *target, ors::Vector &error) {
        ors::Shape *_eef = w.getShapeByName(eef);
        ors::Shape *_target = w.getShapeByName(target);
        return computeAlignmentError(*_eef, *_target, error);
    }

    bool Komo::withinTolerance(ors::Vector &error, ors::Vector &tolerance, int axes) {
        for(uint i = 0; i < 3; i++) if(axes & (1 << i)) {
            if(fabs(error(i)) > tolerance(i))
                return false;
        }
        return true;
    }

    bool Komo::withinTolerance(ors::Vector &error, ors::Vector &tolerance) {
        return withinTolerance(error, tolerance, 7);
    }

    /**
     * Checks if each joint position in given waypoint meets the configured limits
     *
     * @param wp		The waypoint to check
     * @param limits	The limits to enforce
     * @return			True if no joint limit violation was detected
     */
    bool Komo::validateJointLimits(arr wp, arr limits, string &error_msg) {
        CHECK(wp.d0 == limits.d0, "Wrong dimensions!");
        for (int i = 0; i < wp.d0; ++i) {
            double hi = limits(i,1);
            double lo = limits(i,0);

            // ensure that there actually is a limit, i.e. upper-lower > 0
            if(hi == lo)
                continue;

            if(wp(i) < lo || wp(i) > hi) {
                error_msg.append(STRING(boost::format("Joint limit violated for joint '%d' - value: %.3f\n") % i % wp(i)));
                return false;
            }
        }
        return true;
    }

    /**
      Check if all joint positions within given trajectory or configuation are within the
      configured limits
     *
     * @param w The KinematicWorld instance to use
     * @param x The state to check. x can be either a single waypoint (nd=1) or a list of
     *			waypoints(nd=2)
     * @return  True, if no configuration violates the joint limits
     */
    bool Komo::validateJointLimits(ors::KinematicWorld &w, arr x, string &error_msg) {
        // get the joint limits as configured in the ors description
        arr limits = w.getLimits();
        CHECK(x.nd > 0, "Given value is neither a trajectory nor a waypoint!");

        if(x.nd == 2) { // trajectory (list of waypoints)
            // validation steps neccessary for each single point
            for(int i = 0; i < x.d0 - 1; ++i) {
                arr pt = x[i];
                if(!validateJointLimits(pt, limits, error_msg)) {
                    return false;
                }
            }
            return true;

        } else { // single waypoint
            return validateJointLimits(x, limits, error_msg);
        }
    }

    /**
     * Correct joint limit violations in given trajectory. Ensures that each joint position in
     * all waypoints is within configured limits.
     *
     * @param w			The KinematicWorld to use
     * @param x			The trajectory to check
     */
    void Komo::ensureJointLimits(ors::KinematicWorld &w, arr &x) {
        // get the joint limits as configured in the ors description
        arr limits = w.getLimits();
        // steps neccessary for each single point
        for(int i = 0; i < x.d0 - 1; ++i) {
            arr wp = x[i];
            CHECK(wp.d0 == limits.d0, "Wrong dimensions!");
            for (int j = 0; j < wp.d0; ++j) {
                double hi = limits(j,1);
                double lo = limits(j,0);
                // ensure that there actually is a limit, i.e. upper-lower > 0
                if(hi == lo) continue;

                if(wp(j) < lo) {
                    cerr << boost::format("Lower joint limit violated for joint %d in waypoint %d - value: %.3f\n") % j % i % wp(j) << endl;
                    wp(j) = lo;
                    cerr << "corrected!" << endl;
                }

                if(wp(j) > hi){
                    cerr << boost::format("Upper joint limit violated for joint %d in waypoint %d - value: %.3f\n") % j % i % wp(j) << endl;
                    wp(j) = hi;
                    cerr << "corrected!" << endl;
                }
            }
        }
    }

    /**
     * Check given trajectory or waypoint for detected collisions, using given KinematicWorld
     *
     * @param w			The KinematicWorld to use for collision checking
     * @param x			The Trajectory or waypoint to check
     * @param error_msg	A string that will contain a resulting error message
     * @return			True, if no collision was detected
     */
    bool Komo::validateCollisions(ors::KinematicWorld &w, const arr &x, string &error_msg) {
        CHECK(x.nd > 0, "Given value is neither a trajectory nor a waypoint!");
        if(x.nd == 2) { // x is trajectory
            for (int i = 0; i < x.d0 - 1; ++i) {
                arr pt = x[i];
                w.setJointState(pt);
                // force swift to compute the collision proxies...
                w.swift().step(w);
                // then iterate through the list of proxies and make sure that
                // all distances are greater then zero (or an arbitrary collision margin if necessary...)
                for(ors::Proxy *p : w.proxies)
                    if(p->d <= 0) {
                        string sA(w.shapes(p->a)->name);
                        string sB(w.shapes(p->b)->name);

                        error_msg.append(STRING(boost::format("Collision in waypoint %d between '%s' and '%s' detected!\n")	% i % sA % sB));

                        return false;
                    }
            }

            return true;

        } else { // x is single waypoint
            w.setJointState(x);
            // force swift to compute the collision proxies...
            w.swift().step(w);
            // then iterate through the list of proxies and make sure that
            // all distances are greater then zero (or an arbitrary collision margin if necessary...)
            for(ors::Proxy *p : w.proxies)
                if(p->d <= 0) {
                    string sA(w.shapes(p->a)->name);
                    string sB(w.shapes(p->b)->name);

                    error_msg.append(STRING(boost::format("Collision between '%s' and '%s' detected!\n") % sA % sB));

                    cerr << error_msg << endl;
                    w.watch(true, error_msg.c_str());
                    return false;
                }
        }
        return true;
    }

    /**
     * Checks, whether a given goal state was reached
     *
     * @param desired	The desired state
     * @param actual	The actual state
     * @return			True, if actual state is within goal tolerance
     */
    bool Komo::check_goal_state(const arr &desired, const arr &actual) {
        arr error = desired - actual;
        for (int i = 0; i < desired.d0; ++i) {
            if(fabs(error(i)) > 1e-2)
                return false;
        }
        return true;
    }

    void Komo::pathToTrajectory(trajectory_msgs::JointTrajectory &traj, const arr &path) {

        // set joint names
        for (ors::Joint *jnt : _active_joints)
            traj.joint_names.push_back(string(jnt->name));

        // create waypoints
        int num_points = path.d0-1;
        traj.points.resize(num_points);
        for (int i = 0; i < num_points; ++i) {
            const arr &wp = path[i];
            CHECK(wp.N>=traj.joint_names.size(), "Invalid path for given group!");
            trajectory_msgs::JointTrajectoryPoint &point = traj.points[i];
            for (ors::Joint *jnt : _active_joints) {
                point.positions.push_back(wp(jnt->qIndex));
            }
        }

    }

    void Komo::display(bool block, const char *msg) {
        _world->watch(block, msg);
    }

    void Komo::allowContact(const char* link, bool allow) {
        ors::Shape *shape = _world->getShapeByName(link);
        if(shape) {
            shape->cont = !allow;
        } else {
            cerr << "Unable to find shape with name '" << string(link) << "' within model." << endl;
        }
    }

    /****************** end ********************************************/

}
