
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    //console.log(q_robot_config);
    collision_result = kineval.poseIsCollision(q_robot_config);
    //console.log(collision_result);
    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    return robot_collision_forward_kinematics(q);

}



function traverse_collision_forward_kinematics_link(link,mstack,q) {

    /* test collision FK
    console.log(link);
    */
    if (typeof link.visual !== 'undefined') {
        var local_link_xform = matrix_multiply(mstack,generate_translation_matrix(link.visual.origin.xyz[0],link.visual.origin.xyz[1],link.visual.origin.xyz[2]));
    }
    else {
        var local_link_xform = matrix_multiply(mstack,generate_identity());
    }

    // test collision by transforming obstacles in world to link space
/*
    mstack_inv = matrix_invert_affine(mstack);
*/
    mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}

function robot_collision_forward_kinematics(q) {
    
    //偷天换日step1，将机器人c坐标更换为q的值
    var q_temp = {};
    q_temp[0] = robot.origin.xyz[0];
    q_temp[1] = robot.origin.xyz[1];
    q_temp[2] = robot.origin.xyz[2];
    q_temp[3] = robot.origin.rpy[0];
    q_temp[4] = robot.origin.rpy[1];
    q_temp[5] = robot.origin.rpy[2];

    robot.origin.xyz[0] = q[0];
    robot.origin.xyz[1] = q[1];
    robot.origin.xyz[2] = q[2];
    robot.origin.rpy[0] = q[3];
    robot.origin.rpy[1] = q[4];
    robot.origin.rpy[2] = q[5];

    var i7 = 6;
    for (var i6 in robot.joints) {
        q_temp[i6] = robot.joints[i6].angle;
        robot.joints[i6].angle = q[i7];
        i7 += 1;
        //console.log(i6, robot.joints[i6].angle);
    }
    kineval.robotForwardKinematics();
    //console.log(joint_q_temp);
    //alert();
    for (var i5 in robot.links) {
        //if (i == robot.base)
        //    continue;

        var col = collision_FK_link(robot.links[i5], robot.links[i5].xform, q);

        if (col != false) {
            //偷天换日step2，将坐标改回去
            robot.origin.xyz[0] = q_temp[0];
            robot.origin.xyz[1] = q_temp[1];
            robot.origin.xyz[2] = q_temp[2];
            robot.origin.rpy[0] = q_temp[3];
            robot.origin.rpy[1] = q_temp[4];
            robot.origin.rpy[2] = q_temp[5];
            for (var i8 in robot.joints) {
                robot.joints[i8].angle = q_temp[i8];
            }
            kineval.robotForwardKinematics();
            return col;
        }
    }
    //偷天换日step2，将坐标改回去
    robot.origin.xyz[0] = q_temp[0];
    robot.origin.xyz[1] = q_temp[1];
    robot.origin.xyz[2] = q_temp[2];
    robot.origin.rpy[0] = q_temp[3];
    robot.origin.rpy[1] = q_temp[4];
    robot.origin.rpy[2] = q_temp[5];
    for (var i8 in robot.joints) {
        robot.joints[i8].angle = q_temp[i8];
    }
    kineval.robotForwardKinematics();
    return false;
}



function collision_FK_link(link, mstack, q) {

    // this function is part of an FK recursion to test each link 
    //   for collisions, along with a joint traversal function for
    //   the input robot configuration q
    //
    // this function returns the name of a robot link in collision
    //   or false if all its kinematic descendants are not in collision

    // test collision by transforming obstacles in world to link space
    mstack_inv = numeric.inv(mstack);
    // (alternatively) mstack_inv = matrix_invert_affine(mstack);

    var i; var j;

    // test each obstacle against link bbox geometry 
    //   by transforming obstacle into link frame and 
    //   testing against axis aligned bounding box
    for (j in robot_obstacles) {

        var obstacle_local =
          matrix_multiply(mstack_inv, robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true;

        // return false if no collision is detected such that
        //   obstacle lies outside the link extents 
        //   along any dimension of its bounding box
        if (
          (obstacle_local[0][0] <
           (link.bbox.min.x - robot_obstacles[j].radius)
          )
          ||
          (obstacle_local[0][0] >
           (link.bbox.max.x + robot_obstacles[j].radius)
          )
        )
            in_collision = false;

        if (
          (obstacle_local[1][0] <
           (link.bbox.min.y - robot_obstacles[j].radius)
          )
          ||
          (obstacle_local[1][0] >
           (link.bbox.max.y + robot_obstacles[j].radius)
          )
        )
            in_collision = false;

        if (
          (obstacle_local[2][0] <
           (link.bbox.min.z - robot_obstacles[j].radius)
          )
          ||
          (obstacle_local[2][0] >
           (link.bbox.max.z + robot_obstacles[j].radius)
          )
        )
            in_collision = false;

        // return name of link for detected collision if
        //   obstacle lies within the link extents 
        //   along all dimensions of its bounding box
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, 
    //   returning name of descendant link in collision
    //   or false if all descendants are not in collision
    if (typeof link.children !== 'undefined') {
        var local_collision;
        for (i in link.children) {
            // STUDENT: create this joint FK traversal function 
            local_collision =
              collision_FK_joint(robot.joints[link.children[i]], mstack, q)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children
    return false;
}

function collision_FK_joint(joint, mstack_unused, q) {
    var link = joint.child;
    var link_j = robot.links[link];
    var mstack = robot.links[link].xform;
    return collision_FK_link(link_j, mstack, q);
}