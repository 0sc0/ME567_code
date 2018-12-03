/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    else {
        // STENCIL: implement kineval.buildFKTransforms();
        kineval.buildFKTransforms();
    }
}
kineval.buildFKTransforms = function buildFKTransforms() {
    //mat_stack = [];
    //console.log("mat_stack_0", mat_stack);
    /*
    var mat_stack_init = [[[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]];
    console.log("mat_stack_00", mat_stack_init);
    
    mat_stack = [[[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]];

    console.log("mat_stack_01", mat_stack);
    alert();
    */
    stack_pin = 1;
    base_name = "none";

    /*找到base*/
    for (x in robot.links) {
        var x_parent = robot.links[x].parent;
        //console.log(x, x_parent, typeof x_parent);
        if (typeof x_parent === "undefined") {
            //console.log("xform_start", robot.links[x].xform);
            robot.links[x].visited = true;
            base_name = x;
            mat_stack = [];
            break;
        }
    }

    FKstack(base_name);
    //FKstack_debug();

    //console.log(robot.links["base"].xform);
    //alert("stop");
    
    for (z in robot.links) {
        robot.links[z].visited = false;
    }

    //console.log(robot.links["base"].xform);
    kineval.robotDraw()
}

function FKstack(x) {
    if (x == base_name) {
        if (robot.links_geom_imported == true) {
            robot.links[x].xform =
                [[0, 1, 0, 0],
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1]];
        }
        else {
            robot.links[x].xform = [[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]];
        }
        mat_stack.push([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]]);

        if (robot.links_geom_imported == true) {
            var b_position = [0, 0, 0];
            var b_angle = [0, 0, 0];
            b_position[0] = robot.origin.xyz[2];
            b_position[1] = robot.origin.xyz[0];
            b_position[2] = robot.origin.xyz[1];
            b_angle[0] = robot.origin.rpy[2];
            b_angle[1] = robot.origin.rpy[0];
            b_angle[2] = robot.origin.rpy[1];
        }
        else {
            var b_position = robot.origin.xyz;
            var b_angle = robot.origin.rpy;
        }
        

        var bmat_x = generate_rotation_matrix_X(b_angle[0]);
        var bmat_y = generate_rotation_matrix_Y(b_angle[1]);
        var bmat_z = generate_rotation_matrix_Z(b_angle[2]);
        var bmat_t = generate_translation_matrix(b_position[0], b_position[1], b_position[2]);

        var bmat_H = matrix_multiply(matrix_multiply(bmat_t, bmat_z), matrix_multiply(bmat_y, bmat_x));
        robot.links[base_name].xform = matrix_copy(matrix_multiply(robot.links[base_name].xform, bmat_H));
        //正向
        if (robot.links_geom_imported == true) {
            var temp_heading = [[1, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [1, 0, 0, 0]];
        }
        else {
            var temp_heading = [[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [1, 0, 0, 0],
                                [1, 0, 0, 0]];
        }
        temp_heading = matrix_multiply(robot.links[base_name].xform, temp_heading);
        /*
        if (robot.links_geom_imported == true) {
            var back2ROS = [[0, 0, 1, 0],
                            [1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 0]];
            temp_heading = matrix_multiply(back2ROS, temp_heading);
        }*/
        robot_heading = [[temp_heading[0][0]], [temp_heading[1][0]], [temp_heading[2][0]], [temp_heading[3][0]]];
        //侧向
        if (robot.links_geom_imported == true) {
            var temp_lateral = [[0, 0, 0, 0],
                                [1, 0, 0, 0],
                                [0, 0, 0, 0],
                                [1, 0, 0, 0]];
        }
        else {
            var temp_lateral = [[1, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [1, 0, 0, 0]];
        }
        temp_lateral = matrix_multiply(robot.links[base_name].xform, temp_lateral);
        //temp_console = temp_lateral;
        /*
        if (robot.links_geom_imported == true) {
            var back2ROS = [[0, 0, 1, 0],
                            [1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 0]];
            temp_lateral = matrix_multiply(back2ROS, temp_lateral);
        }
        */
        robot_lateral = [[temp_lateral[0][0]], [temp_lateral[1][0]], [temp_lateral[2][0]], [temp_lateral[3][0]]];

    }

    if (robot.links[x].visited === true) {
        for (y in robot.links[x].children) {
            y_name = robot.links[x].children[y];
            //console.log(y_name);
            var next_link = robot.joints[y_name].child;
            if (robot.links[next_link].visited) {
                /*如果全部children都已经visited*/
                if (robot.links[x].children.indexOf(y_name) == robot.links[x].children.length - 1) {
                    /*返回上一个*/
                    var upper_joint = robot.links[x].parent;
                    /*重回base则结束*/
                    if (typeof upper_joint === "undefined"){
                        //console.log("return");
                        return;
                    }
                    else {
                        var upper_link = robot.joints[upper_joint].parent;
                        //console.log(upper_link);
                        mat_stack.pop();
                        FKstack(upper_link);
                    }
                }
                else
                    continue;
            }
            else
                //console.log(next_link);
                FKstack(next_link);
        }
    }

    else {
        //console.log("else");
        var x_parent = robot.links[x].parent;

        var x_position = robot.joints[x_parent].origin.xyz;
        var x_angle = robot.joints[x_parent].origin.rpy;

        var mat_x = generate_rotation_matrix_X(x_angle[0]);
        var mat_y = generate_rotation_matrix_Y(x_angle[1]);
        var mat_z = generate_rotation_matrix_Z(x_angle[2]);
        var mat_t = generate_translation_matrix(x_position[0], x_position[1], x_position[2]);

        var mat_H = matrix_multiply(matrix_multiply(mat_t, mat_z), matrix_multiply(mat_y, mat_x));

        /*continuous和revolute joint角度，转轴*/
        if ((robot.joints[x_parent].type !== 'prismatic')
             && (robot.joints[x_parent].type !== 'fixed')) {
            var j_ang = robot.joints[x_parent].angle;
            var j_axis = robot.joints[x_parent].axis;

            var j_quater = quaternion_from_axisangle(j_axis, j_ang);
            var j_ro_mat = quaternion_to_rotation_matrix(j_quater);

            
            mat_H = matrix_multiply(mat_H, j_ro_mat);
        }
        //prismatic joint延伸
        if (robot.joints[x_parent].type == 'prismatic') {
            var dx = robot.joints[x_parent].axis[0] * robot.joints[x_parent].angle;
            var dy = robot.joints[x_parent].axis[1] * robot.joints[x_parent].angle;
            var dz = robot.joints[x_parent].axis[2] * robot.joints[x_parent].angle;
            
            mat_move = generate_translation_matrix(dx, dy, dz);
            mat_H = matrix_multiply(mat_H, mat_move);
        }

        stack_pin = mat_stack.length;
        mat_stack.push(matrix_multiply(mat_stack[stack_pin - 1], mat_H));

        //console.log(x);
        //console.log(robot.links[base_name].xform);
        robot.links[x].xform = matrix_copy(matrix_multiply(robot.links[base_name].xform, mat_stack[stack_pin]));

        /*joint的xform*/
        //var mat_jx = generate_rotation_matrix_X(Math.acos(robot.joints[x_parent].axis[0]));
        //var mat_jy = generate_rotation_matrix_Y(Math.acos(robot.joints[x_parent].axis[1]));
        //var mat_jz = generate_rotation_matrix_Z(Math.acos(robot.joints[x_parent].axis[2]));
        var mat_jt = [
            [1, 0, 0, robot.links[x].xform[0][3]],
            [0, 1, 0, robot.links[x].xform[1][3]],
            [0, 0, 1, robot.links[x].xform[2][3]],
            [0, 0, 0, 1]
        ]
        //var mat_jH = matrix_multiply(mat_jz, matrix_multiply(mat_jy, mat_jx));

        //var temp_joint_xform = matrix_multiply(robot.links[x].xform, mat_jH);
        //var temp_joint_xform = matrix_copy(robot.links[x].xform);
        //temp_joint_xform[0][1] = 0.707;

        robot.joints[x_parent].xform = matrix_copy(robot.links[x].xform);
        //robot.joints[x_parent].xform = matrix_copy(mat_jt);

        robot.links[x].visited = true;
        /*判断下一步*/
        //console.log("current", x);
        //console.log(robot.links[x].children);
        if (typeof robot.links[x].children === "undefined") {

            /*返回上一级*/
            var upper_joint = robot.links[x].parent;
            var upper_link = robot.joints[upper_joint].parent;
            //console.log(upper_link);
            //console.log("before pop", mat_stack);
            mat_stack.pop();
            //console.log("after pop", mat_stack);
            //alert;
            FKstack(upper_link);
        }
        else {
            /*继续下一级，从第一个children开始*/
            var next_joint = robot.links[x].children[0];
            var next_link = robot.joints[next_joint].child;
            //console.log(next_link);
            FKstack(next_link);
        }
    }
}


    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
/*
var jsmat = [
    [1, 0, 0, 0.5],
    [0, 1, 0, 0.5],
    [0, 0, 1, 0.5],
    [0, 0, 0, 1]
]
var jsmat_turn = matrix_multiply(jsmat, generate_rotation_matrix_X(Math.PI/4));
//console.log(jsmat_turn);
robot.links["clavicle_right"].xform = matrix_copy(jsmat_turn);
*/
/*
if (typeof robot.links[x].xform === "undefined") {
    //console.log(57);
    var mat_base = [
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
    ]
    robot.links[x].xform = matrix_copy(mat_base);
}
else
    break;
*/
function FKstack_debug() {
    y = "base";
    x = "clavicle_right";

    var x_parent = robot.links[x].parent;

    var x_position = robot.joints[x_parent].origin.xyz;
    var x_angle = robot.joints[x_parent].origin.rpy;

    var mat_x = generate_rotation_matrix_X(x_angle[0]);
    var mat_y = generate_rotation_matrix_Y(x_angle[1]);
    var mat_z = generate_rotation_matrix_Z(x_angle[2]);
    var mat_t = generate_translation_matrix(x_position[0], x_position[1], x_position[2]);

    var mat_H = matrix_multiply(matrix_multiply(mat_t, mat_z), matrix_multiply(mat_y, mat_x));

    //stack_pin = mat_stack.length;
    //mat_stack.push(matrix_multiply(mat_stack[stack_pin - 1], mat_H));
    console.log(robot.links[y].xform);
    alert();
    robot.links[y].xform = [
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
    ]
    robot.links[x].xform = matrix_copy(matrix_multiply(mat_H, robot.links[y].xform));
}