
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    //initial trees
    //console.log("start", q_start_config);
    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);

    //initial eps
    eps = 1.5;
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
        var qrand = random_config();
        //console.log("qrand", qrand);
        if (rrt_extend(T_a, qrand) !== "trapped") {
            if (rrt_connect(T_b, qnew) == "reached") {
                rrt_iterate = false;
                return "reached";
            }
        }
        else {
            //console.log("before", T_a, T_b);
            var T_temp = T_a;
            T_a = T_b;
            T_b = T_temp;
            //console.log("after", T_a, T_b);
            return "failed"
        }

        return "extended";
    }
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

function rrt_extend(T, q) {
    //console.log("T", T);
    //console.log("q", q);
    //alert();
    var qnear = nearest_neighbor(T, q);
    //console.log("qnear", qnear);
    qnew = new_config(qnear, q);
    //if (qnew[0] != "NaN") {
    //console.log(qnew);
    //console.log(kineval.poseIsCollision(qnew));
    if (kineval.poseIsCollision(qnew) == false) {
        tree_add_vertex(T, qnew);
        qnear_id = search_tree(qnear, T);
        qnew_id = search_tree(qnew, T);

        tree_add_edge(T, qnear_id, qnew_id);

        cspace_q = q;
        cspace_qnew = qnew;
        //console.log("norm", vec_norm(vec_sub(cspace_q, cspace_qnew)));
        if (rrt_alg < 0.01) {
            if (vec_norm(vec_sub(q_goal_config, cspace_qnew)) < eps) {
                draw_path(T, q, qnew);
                return "reached";
            }
        }

        if (Math.abs(rrt_alg -1) < 0.01) {
            if (vec_norm(vec_sub(cspace_q, cspace_qnew)) < eps) {
                //判断q点是否为另一树的节点
                if (T.vertices[0].vertex == T_a.vertices[0].vertex) {
                    var T_other = T_b;
                }
                else {
                    var T_other = T_a;
                }
                if (search_tree(q, T_other) !== false) {
                    draw_path(T, q, qnear);
                    return "reached";
                }

            }
            else
                return "advanced";
        }
    }
    return "trapped";
}

function rrt_connect(T, q) {
    var S = rrt_extend(T, q);

    while (S == "advanced") {
        S = rrt_extend(T, q);
        //console.log("s", S);
    }
    return S;
}

function random_config() {
    var q_base = new Array(6);

    q_base[0] = robot_boundary[1][0] + Math.random() * (robot_boundary[0][0] - robot_boundary[1][0]);
    q_base[1] = 0;
    q_base[2] = robot_boundary[1][2] + Math.random() * (robot_boundary[0][2] - robot_boundary[1][2]);

    q_base[3] = 0;
    q_base[4] = Math.random() * 2 * Math.PI;
    q_base[5] = 0;

    var q_joints = [];
    var q_joints_index = 0;

    var x;
    for (x in robot.joints) {
        if (robot.joints[x].limit) {
            q_joints[q_joints_index] = robot.joints[x].limit.lower + Math.random() * (robot.joints[x].limit.upper - robot.joints[x].limit.lower);
        }
        else {
            q_joints[q_joints_index] = Math.random() * 2 * Math.PI;
        }
        q_joints_index += 1;
    }

    var q_all = q_base.concat(q_joints);
    return q_all;
}

function new_config(q_nearst, qrand) {
    var step_l = eps;
    //console.log(q_nearst);
    var cspace_q_nearst = q_nearst;
    var cspace_qrand = qrand;
    var vector = vec_sub(cspace_qrand, cspace_q_nearst);
    var vec_nor = vector_normalize(vector);
    //console.log
    var q_new = [];
    for (i4 = 0; i4 < q_nearst.length; i4++) {
        q_new[i4] = q_nearst[i4] + step_l * vec_nor[i4];
    }
    //显示高亮（未定）
    //T.vertices[i].vertex[j].geom.material.color = { r: 1, g: 0, b: 0 };
    //draw_2D_configuration([x_new, y_new]);
    return q_new;
}

function search_tree(q_s, tree_s) {
    //tree.vertices[0].vertex = q;
    //console.log("qs", q_s);
    //console.log("trees", tree_s);
    var ver_n = tree_s.vertices.length;
    for (n = 0; n < ver_n; n++) {
        //console.log(n);
        //console.log("qs", q_s);
        //console.log("tree", tree_s.vertices[n].vertex);
        //console.log("norm", vec_norm(vec_sub(q_s, tree_s.vertices[n].vertex)));
        if (vec_norm(vec_sub(q_s, tree_s.vertices[n].vertex)) < 0.001) {
            //console.log("return", n);
            return n;
        }
    }
    return false;
}

function nearest_neighbor(tree, q_g) {
    var cspace_q = q_g;
    var distance = 10000000000000000;
    var nearest_q
    for (ii = 0; ii < tree.vertices.length; ii++) {
        //console.log("tree", tree);
        //console.log("tree.vertices[i].vertex", tree.vertices[i].vertex);
        var cspace_tree = tree.vertices[ii].vertex;
        var dist_now = vec_norm(vec_sub(cspace_q, cspace_tree));

        //console.log("dist_now", dist_now);
        if (dist_now < distance) {
            distance = dist_now;
            //console.log("i", i);
            //console.log("tree", tree);
            nearest_q = tree.vertices[ii].vertex;
        }
    }

    return nearest_q;
}

function draw_path(T2, q1, q2) {
    if (T2.vertices[0].vertex == T_a.vertices[0].vertex) {
        var T1 = T_b;
    }
    else {
        var T1 = T_a;
    }

    var q2_id = search_tree(q2, T2);
    var path_2 = [];

    while (true) {
        path_2.push(T2.vertices[q2_id]);
        var q2_obj = T2.vertices[q2_id].edges[0];
        q2_id = search_tree(q2_obj.vertex, T2);
        if ((vec_norm(vec_sub(q_goal_config, q2_obj.vertex)) < 0.001) || (vec_norm(vec_sub(q_start_config, q2_obj.vertex)) < 0.001)) {
            path_2.push(T2.vertices[q2_id]);
            break;
        }
    }
    //console.log("alg", rrt_alg);
    if (rrt_alg == 1) {     //rrt_connect
        var q1_id = search_tree(q1, T1);
        var path_1 = [];
        //console.log(T1);
        //console.log(T2);
        while (true) {
            //console.log(T1.vertices[q1_id]);
            path_1.push(T1.vertices[q1_id]);
            var q1_obj = T1.vertices[q1_id].edges[0];
            q1_id = search_tree(q1_obj.vertex, T1);
            if ((vec_norm(vec_sub(q_goal_config, q1_obj.vertex)) < 0.001) || (vec_norm(vec_sub(q_start_config, q1_obj.vertex)) < 0.001)) {
                path_1.push(T1.vertices[q1_id]);
                break;
            }
        }
        path_1.reverse();
        var path_all = path_1.concat(path_2);
        //console.log("draw", path_all);
        drawHighlightedPath(path_all);
    }

    if ((rrt_alg == 0) || (rrt_alg == 2)) {
        var q_init_obj = {};
        var q_goal_obj = {};
        q_init_obj.vertex = q_start_config;
        q_goal_obj.vertex = q_goal_config;
        path_2.push(q_init_obj);
        path_2.unshift(q_goal_obj);
        drawHighlightedPath(path_2);
    }
}

function drawHighlightedPath(path) {
    console.log(path);
    kineval.motion_plan = path;
    for (var i = 0; i < path.length; i++) {
        path[i].geom.material.color = { r: 1, g: 0, b: 0 };
    }

}