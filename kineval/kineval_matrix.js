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

function generate_rotation_matrix_X() {
    var mat = [
        [1, 0, 0, 0]
        [0, Math.cos(z), -Math.sin(z), 0]
        [0, Math.sin(z),  Math.cos(z), 0]
        [0, 0, 0, 1]
    ]
    return mat;
}

function generate_rotation_matrix_Y() {
    var mat = [
        [Math.cos(z), 0, Math.sin(z), 0]
        [0, 1, 0, 0]
        [-Math.sin(z),0, Math.cos(z), 0]
        [0, 0, 0, 1]
    ]
    return mat;
}

function generate_rotation_matrix_Z(z) {
    var mat = [
        [Math.cos(z), -Math.sin(z), 0, 0]
        [Math.sin(z), Math.cos(z), 0, 0]
        [0, 0, 1, 0]
        [0, 0, 0, 1]
    ]
    return mat;
}
    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

