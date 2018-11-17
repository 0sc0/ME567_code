
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code

}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    var H = robot.joints[endeffector_joint].xform;
    var end_pose_w = mat_vec(H, endeffector_position_local);
    //console.log("endeffector_position_local", endeffector_position_local);
    end_pose_w.pop();
    var end_angle_w = mat_vec(H, [0, 0, 1, 0]);
    end_angle_w.pop();
    var end_vec_w = end_pose_w.concat(end_angle_w);
	//var end_vec_w = end_pose_w.concat([0,0,0]);
    //console.log("end_vec_w", end_vec_w);

    var J = jacobian(endeffector_joint, end_pose_w);
    //console.log("jacobian", J);
    //alert();
    //console.log("endeffector_target_world", endeffector_target_world);
    var end_target_w = endeffector_target_world.position.slice(0, 3).concat(endeffector_target_world.orientation);
	
    //console.log("end_target_w", end_target_w);
	//alert();
    //console.log("end_vec_w",end_vec_w);
    var J_T = matrix_transpose(J);
	
    var delta_x = vec_sub(end_target_w, end_vec_w);
	delta_x[3] = 0;
	delta_x[4] = 0;
	delta_x[5] = 0;

    if (kineval.params.ik_pseudoinverse) {
        var J_JT = matrix_multiply(J, J_T);
        //console.log("J", J);
        //console.log("JT", J_T);
        //console.log("J_JT", J_JT);
        //alert();
        var J_JT_I = numeric.inv(J_JT);
        
        var JT_J_JT_I = matrix_multiply(J_T, J_JT_I);
        var delta_theta = mat_vec(JT_J_JT_I, delta_x);
        
    }
    else {
        //console.log("JT", J_T);
        //console.log("dx", delta_x);
        var delta_theta = mat_vec(J_T, delta_x);
        //console.log("delta_theta", delta_theta);
		//alert();
    }
    //
    //console.log("delta_x", delta_x);
    //console.log("delta_theta", delta_theta);

    //for (i = 0; i < delta_theta.length; i++) {
    //    delta_theta[i] = kineval.params.ik_steplength * delta_theta[i];
    //}
    //console.log("delta_theta", delta_theta);
    

    x = endeffector_joint;
    i = 0;
    while (x !== "undefined") {
        //console.log("x", x);
        //console.log(i, delta_theta[i]);
        if (robot.joints[x].type == "fixed") {
            x = robot.links[robot.joints[x].parent].parent;
            continue;
        }

        robot.joints[x].control = kineval.params.ik_steplength * delta_theta[i];

        x = robot.links[robot.joints[x].parent].parent;
        i += 1;
        if (typeof x == "undefined")
            break;
    }
    //alert();
}

function jacobian(endeffector_joint, end_pose_w) {
    //var end_pose_mat = generate_translation_matrix(robot.endeffector.position[0], robot.endeffector.position[1], robot.endeffector.position[2]);
    

    J = new Array(6);
    J[0] = new Array();
    J[1] = new Array();
    J[2] = new Array();
    J[3] = new Array();
    J[4] = new Array();
    J[5] = new Array();

    x = endeffector_joint
    while (x !== "undefined") {
        //console.log("type", robot.joints[x].type);
        
        if (robot.joints[x].type == "fixed") {
            x = robot.links[robot.joints[x].parent].parent;
            continue;
        }
        
        var Jx = J_vector(x, end_pose_w);

        J[0].push(Jx[0]);
        J[1].push(Jx[1]);
        J[2].push(Jx[2]);
        J[3].push(Jx[3]);
        J[4].push(Jx[4]);
        J[5].push(Jx[5]);
        /*
        console.log("x", x);
        console.log("robot.joints[x].parent", robot.joints[x].parent);
        alert();
        */
        x = robot.links[robot.joints[x].parent].parent;
        
        if (typeof x == "undefined") {
            
            break;
        }
            
    }
	//console.log("Jacobian", J);
	//alert();
    return J;

}

function J_vector(x, end_pose_w) {
    var H_j = robot.joints[x].xform;
    var joint_pose_l = [robot.joints[x].origin.xyz[0], robot.joints[x].origin.xyz[1], robot.joints[x].origin.xyz[2], 1];
    var joint_pose_w = mat_vec(H_j, [0, 0, 0, 1]);
    var joint_axis_l = [robot.joints[x].axis[0], robot.joints[x].axis[1], robot.joints[x].axis[2], 1];
    var joint_axis_w = mat_vec(H_j, joint_axis_l);
    joint_pose_w.pop();
    joint_axis_w.pop();
	joint_axis_w = vec_sub(joint_axis_w, joint_pose_w);
    if (robot.joints[x].type == "prismatic") {
        var Jx = [joint_axis_w[0], joint_axis_w[1], joint_axis_w[2]];
        Jx.push(0, 0, 0);
    }
    else {
        var Jx = vector_cross(joint_axis_w, vec_sub(end_pose_w, joint_pose_w));
        
        Jx.push(joint_axis_w[0], joint_axis_w[1], joint_axis_w[2]);
        /*
        console.log("joint_pose_w", joint_pose_w);
        console.log("end_pose_w", end_pose_w);
        console.log("joint_axis_w", joint_axis_w);
        console.log("z", vec_sub(joint_axis_w, joint_pose_w));
        console.log("dO", vec_sub(end_pose_w, joint_pose_w));
        */
    }
    //console.log("joint", x);
    //console.log("Jx", Jx);
    return Jx;
}