//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

function quaternion_from_axisangle(axis, angle) {
    var quater = [Math.cos(angle / 2), axis[0] * Math.sin(angle / 2), axis[1] * Math.sin(angle / 2), axis[2] * Math.sin(angle / 2)];
    var nor_quater = quaternion_normalize(quater);
    return nor_quater;
}

function quaternion_normalize(q) {
    var norm2 = Math.pow(q[0], 2) + Math.pow(q[1], 2) + Math.pow(q[2], 2) + Math.pow(q[3], 2);
    var norm = Math.pow(norm2, 0.5);
    var quater = [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
    return quater;
}

function quaternion_to_rotation_matrix(q) {
    var q0 = q[0];
    var q1 = q[1];
    var q2 = q[2];
    var q3 = q[3];

    var a00 = Math.pow(q0, 2) + Math.pow(q1, 2) - Math.pow(q2, 2) - Math.pow(q3, 2);
    var a01 = 2 * (q1 * q2 - q0 * q3);
    var a02 = 2 * (q0 * q2 + q1 * q3);
    var a10 = 2 * (q1 * q2 + q0 * q3);
    var a11 = Math.pow(q0, 2) - Math.pow(q1, 2) + Math.pow(q2, 2) - Math.pow(q3, 2);
    var a12 = 2 * (q2 * q3 - q0 * q1);
    var a20 = 2 * (q1 * q3 - q0 * q2);
    var a21 = 2 * (q0 * q1 + q2 * q3);
    var a22 = Math.pow(q0, 2) - Math.pow(q1, 2) - Math.pow(q2, 2) + Math.pow(q3, 2);

    var rotation_matrix = [
        [a00, a01, a02, 0],
        [a10, a11, a12, 0],
        [a20, a21, a22, 0],
        [0,0,0,1]
    ];
    return rotation_matrix;
}

function quaternion_multiply(q1,q2) {
    var a = q1[0];
    var b = q1[1];
    var c = q1[2];
    var d = q1[3];

    var e = q2[0];
    var f = q2[1];
    var g = q2[2];
    var h = q2[3];

    var multi0 = a * e - b * f - c * g - d * h;
    var multi1 = a * f + b * e + c * h - d * g;
    var multi2 = a * g - b * h + c * e + d * f;
    var multi3 = a * h + b * g - c * f + d * e;

    var multi = [multi0, multi1, multi2, multi3];

    return multi;
}