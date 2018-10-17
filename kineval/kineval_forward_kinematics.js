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
    
    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();
}
kineval.buildFKTransforms = function buildFKTransforms() {

    mat_stack = [
                        [
                            [1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]
                        ]
    ]
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
            break;
        }
    }

    FKstack(base_name);
    

    //console.log(robot.links["base"].xform);
    //alert("stop");
    

    //console.log(robot.links["base"].xform);
    kineval.robotDraw()
}

function FKstack(x) {
    var finish = false;
    //console.log(robot.links[x].visited);
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

        stack_pin = mat_stack.length;
        mat_stack.push(matrix_multiply(mat_stack[stack_pin - 1], mat_H));

        robot.links[x].xform = matrix_copy(matrix_multiply(mat_stack[stack_pin], robot.links[base_name].xform));
        robot.links[x].visited = true;
        /*判断下一步*/
        console.log("current", x);
        console.log(robot.links[x].children);
        if (typeof robot.links[x].children === "undefined") {
            mat_stack.pop();
            /*返回上一级*/
            var upper_joint = robot.links[x].parent;
            var upper_link = robot.joints[upper_joint].parent;
            console.log(upper_link);
            FKstack(upper_link);
        }
        else {
            /*继续下一级，从第一个children开始*/
            var next_joint = robot.links[x].children[0];
            var next_link = robot.joints[next_joint].child;
            console.log(next_link);
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