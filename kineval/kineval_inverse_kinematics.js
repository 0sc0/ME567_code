
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
    kineval.params.trial_ik_random.start.getTime();



    // get endeffector Cartesian position in the world

    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform, robot.endeffector.position);



    // compute distance of endeffector to target

    kineval.params.trial_ik_random.distance_current = Math.sqrt(

            Math.pow(kineval.params.ik_target.position[0][0] - endeffector_world[0][0], 2.0)

            + Math.pow(kineval.params.ik_target.position[1][0] - endeffector_world[1][0], 2.0)

            + Math.pow(kineval.params.ik_target.position[2][0] - endeffector_world[2][0], 2.0));



    // if target reached, increment scoring and generate new target location

    // KE 2 : convert hardcoded constants into proper parameters

    if (kineval.params.trial_ik_random.distance_current < 0.01) {

        kineval.params.ik_target.position[0][0] = 1.2 * (Math.random() - 0.5);

        kineval.params.ik_target.position[1][0] = 1.2 * (Math.random() - 0.5) + 1.5;

        kineval.params.ik_target.position[2][0] = 0.7 * (Math.random() - 0.5) + 0.5;

        kineval.params.trial_ik_random.targets += 1;

        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;

    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    var H = robot.joints[endeffector_joint].xform;
    var end_pose_w = mat_vec(H, endeffector_position_local);
    //console.log("endeffector_position_local", endeffector_position_local);
    end_pose_w.pop();
    //var end_direction_w = mat_vec(H, [0, 0, 1, 0]);
    //end_direction_w.pop();
    //var end_angle_w = vec2angle(end_direction_w);
    //console.log("H54", H);

    var mat_y90 = generate_rotation_matrix_Y(Math.PI / 2);
    var end_H = matrix_multiply(H, mat_y90);
    var end_angle_w = xform2angle(end_H);
    
    //end_angle = mat_vec(mat_y90, end_angle_w);
    var end_vec_w = end_pose_w.concat(end_angle_w);
	//var end_vec_w = end_pose_w.concat([0,0,0]);
    //console.log("end_vec_w", end_vec_w);

    var J = jacobian(endeffector_joint, end_pose_w);
    //console.log("jacobian", J);
    //alert();
    //console.log("endeffector_target_world", endeffector_target_world);
    var mat_x = generate_rotation_matrix_X(endeffector_target_world.orientation[0]);
    var mat_y = generate_rotation_matrix_Y(endeffector_target_world.orientation[1]);
    var mat_z = generate_rotation_matrix_Z(endeffector_target_world.orientation[2]);
    
    var mat_xyz = matrix_multiply(mat_x, matrix_multiply(mat_y, mat_z));
    //var mat_xyz = matrix_multiply(matrix_multiply(mat_x, mat_y), matrix_multiply(mat_z, mat_x90));
    //var end_dirction = mat_vec(mat_xyz);
    var end_angle = xform2angle(mat_xyz);
    //var end_angle = end_dirction;
    //end_angle = endeffector_target_world.orientation;
    //end_angle.pop();
    var end_position = [endeffector_target_world.position[0][0], endeffector_target_world.position[1][0], endeffector_target_world.position[2][0]];
    var end_target_w = end_position.concat(end_angle);
	
    //console.log("end_target_w", end_target_w);
	//alert();
    //console.log("end_vec_w",end_vec_w);
    
	
    var delta_x = vec_sub(end_target_w,end_vec_w);
    /*
	delta_x[3] = 0;
	delta_x[4] = 0;
	delta_x[5] = 0;
    */
    if (kineval.params.ik_pseudoinverse) {
        var pseudo_I = matrix_pseudoinverse(J)

        //var test1 = matrix_multiply(J, pseudo_I);
        //var test2 = matrix_multiply(pseudo_I, J);

        var delta_theta = mat_vec(pseudo_I, delta_x);
        
    }
    else {
        //console.log("JT", J_T);
        //console.log("dx", delta_x);
        var J_T = matrix_transpose(J);
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
    
    if (kineval.params.ik_orientation_included) {
        J = new Array(6);
        J[0] = new Array();
        J[1] = new Array();
        J[2] = new Array();
        J[3] = new Array();
        J[4] = new Array();
        J[5] = new Array();
    }
    else {
        J = new Array(3);
        J[0] = new Array();
        J[1] = new Array();
        J[2] = new Array();
    }


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
        if (kineval.params.ik_orientation_included) {
            J[3].push(Jx[3]);
            J[4].push(Jx[4]);
            J[5].push(Jx[5]);
        }
        
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

    //var ang = vec2angle([1, -1, -1]);
	//console.log("ang", ang);
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
	    if (kineval.params.ik_orientation_included)
            Jx.push(0, 0, 0);
    }
    else {
        var Jx = vector_cross(joint_axis_w, vec_sub(end_pose_w, joint_pose_w));
        if (kineval.params.ik_orientation_included)
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

function xform2angle(xform) {
    //console.log("xform", xform);
    //alert();
    var tan_x = xform[2][1] / xform[2][2];
    var tan_y = -xform[2][0] / Math.pow(Math.pow(xform[2][1], 2) + Math.pow(xform[2][2], 2), 0.5);
    var tan_z = xform[1][0]/xform[0][0];
    /*
    if (xform[2][2] < 0.0001)
        var theta_x = Math.PI / 2;
    else
        var theta_x = Math.atan(tan_x);

    if (Math.pow(Math.pow(xform[2][1], 2) + Math.pow(xform[2][2], 2), 0.5) < 0.0001)
        var theta_y = Math.PI / 2;
    else
        var theta_y = Math.atan(tan_y);

    if (xform[0][0] < 0.0001)
        var theta_z = Math.PI / 2;
    else
        var theta_z = Math.atan(tan_z);
    */
    var theta_y = Math.atan2(-xform[2][0], Math.pow(Math.pow(xform[2][1], 2) + Math.pow(xform[2][2], 2), 0.5));

    if ((theta_y >= -Math.PI) && (theta_y <= Math.PI)) {
        var theta_z = Math.atan2(xform[1][0], xform[0][0]);
        var theta_x = Math.atan2(xform[2][1], xform[2][2]);
    }
    else {
        var theta_z = Math.atan2(-xform[1][0], -xform[0][0]);
        var theta_x = Math.atan2(-xform[2][1], -xform[2][2]);
    }

    return [theta_x, theta_y, theta_z];
}

function vec2angle(vec) {
    if (vec.length == 4)
        vec.pop();
    
    var xyz_w = [[1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]]
    var xyz_angle = new Array(3);

    for (j = 0; j < 3; j++) {
        //console.log(j);
        xyz_angle[j] = cal_angle(vec, xyz_w[j]);
    }
    //console.log(xyz_angle);
    //alert();

    return xyz_angle;
}

function cal_angle(vec, base) {
    vec = vector_normalize(vec);
    base = vector_normalize(base);

    //console.log("vec",vec);
    //console.log("base", base);

    var l = vec.length;
    var vec_dot = 0;
    for (i = 0; i < l; i++) {
        vec_dot += vec[i] * base[i];
    }

    var angle = Math.acos(vec_dot);
    /*
    var vec_cross = vector_cross(base, vec);
    var base_vec_cross = vector_cross(base, vec_cross);
    var vec_dot_base_vec_cross = 0;
    for (i = 0; i < 3; i++) {
        vec_dot_base_vec_cross += vec[i] * base_vec_cross[i];
    }

    if (vec_dot_base_vec_cross > 0)
        angle = -angle;
    */
    return angle;
}