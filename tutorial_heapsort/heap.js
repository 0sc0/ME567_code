/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
//length是heap的最大id

function heap_adjust(heap, ip_id) {
    var parent = Math.floor((ip_id-1)/2);
    var changed = 0;
    var temp
    var next_id


    if (ip_id != 0 && parent >= 0 && heap[ip_id] > heap[parent]) {
        temp = heap[ip_id];
        heap[ip_id] = heap[parent];
        heap[parent] = temp;
        next_id = parent;
        changed = 1;
    }

    if (changed == 1) {
        heap_adjust(heap, parent);
    }
}


function minheap_insert(heap, new_element) {
    // STENCIL: implement your min binary heap insert operation
    var i = 0;
    var l = heap.length;
    
   // console.log("333 " + heap);
   // console.log("333 " + []);


    if (l == 0) {
        heap[0] = new_element;
        console.log("333 " + heap);
    }
    else {
        
       // console.log(l);
       // console.log(heap);
        heap[l] = new_element;
        heap_adjust(heap, l);
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var temp;
    var l = heap.length;

    temp = heap[0];
    /*
    if (l == 1) {
        return temp;
    }
    */
    parent_son(heap, 0);
    
    return temp;
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
// STENCIL: ensure extract method is within minheaper object

function parent_son(heap, ip_id) {
    var son_l_id = 2 * ip_id + 1;
    var son_r_id = 2 * ip_id + 2;
    var l = heap.length;

    if ((son_r_id >= l) && (son_l_id < l)) {
        heap[ip_id] = heap[son_l_id];
        heap.pop();
        return 0;
    }

    if (son_l_id >= l) {
        heap[ip_id] = heap[l - 1];
        heap_adjust(heap, ip_id)
        heap.pop();
        return 0;
    }
    else {
        if (heap[son_l_id] > heap[son_r_id]) {
            heap[ip_id] = heap[son_l_id];
            parent_son(heap, son_l_id);
        }
        else {
            heap[ip_id] = heap[son_r_id];
            parent_son(heap, son_r_id);
        }
    }
}






