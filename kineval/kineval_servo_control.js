
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    //kineval.setpoints =[{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0.04,"l_gripper_finger_joint":0.04,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0.7169999999999997,"upperarm_roll_joint":0,"elbow_flex_joint":-2.25,"forearm_roll_joint":0,"wrist_flex_joint":-0.8999999999999999,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0.7169999999999997,"upperarm_roll_joint":0,"elbow_flex_joint":-2.25,"forearm_roll_joint":1.7000000000000004,"wrist_flex_joint":0.7,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0.201,"shoulder_pan_joint":1.2,"shoulder_lift_joint":-0.28300000000000036,"upperarm_roll_joint":0,"elbow_flex_joint":-1.149999999999999,"forearm_roll_joint":3.3000000000000016,"wrist_flex_joint":1.4000000000000001,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0.04,"l_gripper_finger_joint":0.04,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0.04,"l_gripper_finger_joint":0.04,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0.04,"l_gripper_finger_joint":0.04,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0}]
    kineval.setpoints = [{ "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "upperarm_roll_joint": 0, "elbow_flex_joint": 0, "forearm_roll_joint": 0, "wrist_flex_joint": 0, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0.04, "l_gripper_finger_joint": 0.04, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0.7169999999999997, "upperarm_roll_joint": 0, "elbow_flex_joint": -2.25, "forearm_roll_joint": 0, "wrist_flex_joint": -0.8999999999999999, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0, "l_gripper_finger_joint": 0, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0.7169999999999997, "upperarm_roll_joint": 0, "elbow_flex_joint": -2.25, "forearm_roll_joint": 1.7000000000000004, "wrist_flex_joint": 0.7, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0, "l_gripper_finger_joint": 0, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 1.2, "shoulder_lift_joint": -0.28300000000000036, "upperarm_roll_joint": 0, "elbow_flex_joint": -1.149999999999999, "forearm_roll_joint": 3.3000000000000016, "wrist_flex_joint": 1.4000000000000001, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0, "l_gripper_finger_joint": 0, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "upperarm_roll_joint": 0, "elbow_flex_joint": 0, "forearm_roll_joint": 0, "wrist_flex_joint": 0, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0.04, "l_gripper_finger_joint": 0.04, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "upperarm_roll_joint": 0, "elbow_flex_joint": 0, "forearm_roll_joint": 0, "wrist_flex_joint": 0, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0, "l_gripper_finger_joint": 0, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "upperarm_roll_joint": 0, "elbow_flex_joint": 0, "forearm_roll_joint": 0, "wrist_flex_joint": 0, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0.04, "l_gripper_finger_joint": 0.04, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "upperarm_roll_joint": 0, "elbow_flex_joint": 0, "forearm_roll_joint": 0, "wrist_flex_joint": 0, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0, "l_gripper_finger_joint": 0, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "upperarm_roll_joint": 0, "elbow_flex_joint": 0, "forearm_roll_joint": 0, "wrist_flex_joint": 0, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0.04, "l_gripper_finger_joint": 0.04, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }, { "torso_lift_joint": 0, "shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "upperarm_roll_joint": 0, "elbow_flex_joint": 0, "forearm_roll_joint": 0, "wrist_flex_joint": 0, "wrist_roll_joint": 0, "gripper_axis": 0, "head_pan_joint": 0, "head_tilt_joint": 0, "torso_fixed_joint": 0, "r_wheel_joint": 0, "l_wheel_joint": 0, "r_gripper_finger_joint": 0, "l_gripper_finger_joint": 0, "bellows_joint": 0, "bellows_joint2": 0, "estop_joint": 0, "laser_joint": 0 }]
    // STENCIL: implement FSM to cycle through dance pose setpoints
    var right_pose = true;
    var pose_id = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];

    kineval.setPoseSetpoint(pose_id);

    //console.log(right_pose);
    for (x in robot.joints) {
        right_pose = right_pose && (Math.abs(kineval.setpoints[pose_id][x] - robot.joints[x].angle) < 0.1);
        /*console.log()
        if (!(Math.abs(kineval.setpoints[pose_id][x] - robot.joints[x].angle) < 0.1)) {
            console.log(x, kineval.setpoints[pose_id][x], robot.joints[x].angle);
        }*/
    }
    if (right_pose) {
        if (kineval.params.dance_pose_index >= kineval.params.dance_sequence_index.length-1)
            kineval.params.dance_pose_index = 0;
        else
            kineval.params.dance_pose_index += 1;
        kineval.setpointDanceSequence();
    }
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    
    for (x in robot.joints) {
        var dont_have_limit = ((typeof robot.joints[x].limit) == "undefined");
        if(!dont_have_limit){
            var fit_limit = (robot.joints[x].angle > robot.joints[x].limit.lower) && (robot.joints[x].angle < robot.joints[x].limit.upper);
        }
        else
            var fit_limit = false;

        if (dont_have_limit || fit_limit) {
            var error = kineval.params.setpoint_target[x] - robot.joints[x].angle;
            //console.log("error", error);
            robot.joints[x].control = robot.joints[x].servo.p_gain * error;
        }
        else
            robot.joints[x].control = 0;
    }
}


