//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    //console.log(typeof matrix_multiply);
    return mat;
}

function matrix_multiply(m1, m2) {
    var mat = [];
    var i, j;
    var x, y;
    //console.log(25);
    //console.log(m1);
    //console.log(m2);

    for (i = 0; i < m1.length; i++) {
        mat[i] = [];
        for (j = 0; j < m2[0].length; j++) {
            mat[i][j] = 0;
            for (x = 0; x < m1[0].length; x++){
                    //console.log(m1[i]);
                    //console.log("m1", i, x);
                    //console.log("m2", y, j);
                    mat[i][j] += m1[i][x] * m2[x][j];
                    //console.log("mat", mat[i][j]);
                
            }
        }
    }
    //console.log(mat);
    //alert("before error.");
    return mat;
}

function generate_translation_matrix(dx, dy, dz) {
    var mat = [
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ]
    return mat;
}

function generate_rotation_matrix_X(z) {
    var mat = [
        [1, 0, 0, 0],
        [0, Math.cos(z), -Math.sin(z), 0],
        [0, Math.sin(z),  Math.cos(z), 0],
        [0, 0, 0, 1]
    ]
    return mat;
}

function generate_rotation_matrix_Y(z) {
    var mat = [
        [Math.cos(z), 0, Math.sin(z), 0],
        [0, 1, 0, 0],
        [-Math.sin(z),0, Math.cos(z), 0],
        [0, 0, 0, 1]
    ]
    return mat;
}

function generate_rotation_matrix_Z(z) {
    var mat = [
        [Math.cos(z), -Math.sin(z), 0, 0],
        [Math.sin(z), Math.cos(z), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
    return mat;
}

function vector_cross(v1, v2) {
    var vector = [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]
    ]

    return vector;
}

function vector_normalize(v) {
    var l = v.length;
    var module = 0;

    for (i = 0; i < l; i++) {
        module += Math.pow(v[i], 2)
    }
    module = Math.pow(module, 0.5);

    var vector = new Array(l);
    for (i = 0; i < l; i++) {
        vector[i] = v[i] / module;
    }

    return vector;
}

function vec_sub(v1, v2) {
    var out_vec = [];
    for (i = 0; i < v1.length; i++) {
        out_vec[i] = v1[i] - v2[i];
    }

    return out_vec;
}

function mat_vec(mat, vec) {
    //console.log("mat",mat);
    //console.log("vec", vec);
    //console.log(mat[0][0] * vec[0], mat[0][1] * vec[1], mat[0][2] * vec[2], mat[0][3] * vec[3]);
    var i = mat.length;
    var j = mat[0].length;

    var out_vec = new Array(i);

    for (xx = 0; xx < i; xx++) {
        out_vec[xx] = 0;
    }

    for (xx = 0; xx < i; xx++) {
        for(yy=0;yy<j;yy++){
            out_vec[xx] += mat[xx][yy]*vec[yy];
        }
    }
    /*
    var out_vec = [
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2] + mat[0][3] * vec[3],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2] + mat[1][3] * vec[3],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2] + mat[2][3] * vec[3],
        mat[3][0] * vec[0] + mat[3][1] * vec[1] + mat[3][2] * vec[2] + mat[3][3] * vec[3],
    ];
    */
    //console.log("out",out_vec);
    return out_vec;
}

function matrix_transpose(mat) {
    var i = mat.length;
    var j = mat[0].length;

    var out_mat = new Array(j);
    for (z = 0; z < j; z++){
        out_mat[z] = new Array();
    }

    for (x = 0; x < j; x++) {
        for (y = 0; y < i; y++) {
            out_mat[x][y] = mat[y][x];
        }
    }

    return out_mat;
}

function matrix_pseudoinverse(J) {
    var J_T = matrix_transpose(J);

    if (J.length > J[0].length) {
        var JT_J = matrix_multiply(J_T, J);
        var JT_J_I = numeric.inv(JT_J);
        var pseudo_I = matrix_multiply(JT_J_I, J_T);
    }
    if (J.length < J[0].length) {
        var J_JT = matrix_multiply(J, J_T);
        var J_JT_I = numeric.inv(J_JT);
        var pseudo_I = matrix_multiply(J_T, J_JT_I);
    }
    return pseudo_I;
}

function vec_norm(vec) {
    var l = vec.length;
    var sum2 = 0;
    for (i = 0; i < l; i++) {
        sum2 += Math.pow(vec[i], 2);
    }
    return Math.pow(sum2, 0.5);
}
    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply                done
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize               done
    //   vector_cross                   done
    //   generate_identity
    //   generate_translation_matrix    done
    //   generate_rotation_matrix_X     done
    //   generate_rotation_matrix_Y     done
    //   generate_rotation_matrix_Z     done

