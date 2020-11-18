function f_hello() {
    console.log("hello");
}

function f_get_hr_spo2(ir, rd) {
	// calculates DC mean and subtract DC from ir
    ir_mean = 0;
    buff_len = ir.length;
    for (var i=0; i<buff_len; i++) {
    	ir_mean += ir[i];
    }    
    //console.log(ir_mean/buff_len);
}