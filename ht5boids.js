// HT5Boids

//Copyright (c) 2014 Peter Corbett

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


// Set up HTML 5 stuff
var canvas = document.getElementById('canvas');
var ctx = canvas.getContext('2d');

// Generation
// Arena width and height
var gwidth = 800;
var gheight = 600;

var nframes = 0;

var items = [];
var nitems = 50;

// Degrees radians
var d2r = Math.PI / 180;
var r2d = 1 / d2r;

// Parameters
var bn = [];
var bc = [];
var bspeed = [];
var bonly = [];
var bmturn = [];
var bedged = [];
var bedgem = [];
var bedgemt = [];
var bcold = [];
var bcolm = [];
var bcolmt = [];
var btfm = [];
var btfmt = [];
var btffm = [];
var btffmt = [];
var btfft = [];
var bmhm = [];
var bmhmt = [];
var blfn = [];
var blfmd = [];
var brtg = [];
var brtc = [];

// Read the HTML form
function readform() {
	gwidth = parseFloat(document.getElementById("gwidth").value);
	gheight = parseFloat(document.getElementById("gheight").value);

	for(var i=0;i<4;i++) {
		bn[i] = parseInt(document.getElementById("b" + i + "n").value);
		bc[i] = document.getElementById("b" + i + "c").value;
		bspeed[i] = parseFloat(document.getElementById("b" + i + "speed").value);
		bonly[i] = document.getElementById("b" + i + "only").checked ? 1 : 0;
		bmturn[i] = parseFloat(document.getElementById("b" + i + "mturn").value);
		bedged[i] = parseFloat(document.getElementById("b" + i + "edged").value);
		bedgem[i] = parseFloat(document.getElementById("b" + i + "edgem").value);
		bedgemt[i] = parseFloat(document.getElementById("b" + i + "edgemt").value);
		bcold[i] = parseFloat(document.getElementById("b" + i + "cold").value);
		bcolm[i] = parseFloat(document.getElementById("b" + i + "colm").value);
		bcolmt[i] = parseFloat(document.getElementById("b" + i + "colmt").value);
		btfm[i] = parseFloat(document.getElementById("b" + i + "tfm").value);
		btfmt[i] = parseFloat(document.getElementById("b" + i + "tfmt").value);
		btfft[i] = parseFloat(document.getElementById("b" + i + "tfft").value);
		btffm[i] = parseFloat(document.getElementById("b" + i + "tffm").value);
		btffmt[i] = parseFloat(document.getElementById("b" + i + "tffmt").value);
		bmhm[i] = parseFloat(document.getElementById("b" + i + "mhm").value);
		bmhmt[i] = parseFloat(document.getElementById("b" + i + "mhmt").value);
		blfn[i] = parseFloat(document.getElementById("b" + i + "lfn").value);
		blfmd[i] = parseFloat(document.getElementById("b" + i + "lfmd").value);
		brtc[i] = parseFloat(document.getElementById("b" + i + "rtc").value);
		brtg[i] = parseFloat(document.getElementById("b" + i + "rtg").value);
	}

}

// Some initial clearing-up which is unlikely to have a huge effect
function presetup() {
	canvas.width = gwidth;
	canvas.height = gheight;
	nframes = 0;
	items = [];
}

// Setup - this could vary from version to version
function setup() {
	var id = 0;
	for(var j=0;j<bn.length;j++) {
		for(var i=0;i<bn[j];i++) {
			var item = {id:id,
						x:Math.random()*canvas.width,
						y:Math.random()*canvas.height,
						dir:Math.random()*360,
						type:j};
			items.push(item);
			id++;
		}
	}
}

// Some standard end-of-setup functions... Seem not to be necessary here.
function endsetup() {
} 

// Box-Muller Transform Gaussian
function rnd_gauss() {
	return Math.sqrt(-2 * Math.log(Math.random())) * Math.cos(2 * Math.PI * Math.random());
}

// A random number from a Cauchy distribution
function rnd_cauchy() {
	return rnd_gauss() / rnd_gauss();
}

// Update a frame
function update() {
	var tt = Date.now();
	var i;
	var j;
	var item;

	// For each item, calculate which way it's pointing in Cartesian terms, so you can average it.
	
	for(i=0;i<items.length;i++) {
		item = items[i];
		item.dx = Math.sin(item.dir * d2r);
		item.dy = -Math.cos(item.dir * d2r);
	}
	
	// For each item, calculate behaviour
	for(i=0;i<items.length;i++) {
		item = items[i];
		var turn = 0;
		
		
		// Find the closest boids (of the right type, not too far...) and keep a log of distances
		
		var dl = [];
		var closest = [];
		var lfmd = blfmd[item.type];
		for(j=0;j<items.length;j++) {
			if(i != j) {
				var i2 = items[j];
				var dist = Math.sqrt(((item.x-i2.x)*(item.x-i2.x))+((item.y-i2.y)*(item.y-i2.y)));
				dl.push(dist);
				if((bonly[item.type] == 0 || item.type == i2.type) && dist <= lfmd) closest.push(j);
			} else {
				dl.push(100000);
			}
		}

		var heap = [];
		var hmax = blfn[item.type];
		// Heap sort, using min heap, keeping only needed items. This avoids having to sort a list
		// and then throw most of it away.
		for(var j=0;j<closest.length;j++) {
			// Insert into heap
			var idx = heap.length;
			heap.push(closest[j]);
			var parent = Math.floor((idx-1)/2);
			var tmp = 0;
			while(idx > 0 && dl[heap[idx]] > dl[heap[parent]]) {
				tmp = heap[idx];
				heap[idx] = heap[parent];
				heap[parent] = tmp;
				idx = parent;
				parent = Math.floor((idx-1)/2);
			}
			// If the heap is full, throw away the furthest member
			if(heap.length > hmax) {
				idx = heap.length - 1;
				heap[0] = heap[idx];
				heap.pop();
				idx = 0;
				var ch1 = (idx*2)+1;
				var ch2 = ch1 + 1;
				var inloop = 1;
				while(inloop == 1) {
					var largest = idx;
					if(ch1 < heap.length && dl[heap[ch1]] > dl[heap[largest]]) {
						largest = ch1;
					}
					if(ch2 < heap.length && dl[heap[ch2]] > dl[heap[largest]]) {
						largest = ch2;
					}
					if(largest == idx) {
						inloop = 0;
					} else {
						tmp = heap[largest];
						heap[largest] = heap[idx];
						heap[idx] = tmp;
						idx = largest;
						ch1 = (idx*2)+1;
						ch2 = ch1 + 1;
					}
				}
			}
		}
		//closest.sort(function(a,b) {return dl[a]-dl[b]});
		closest = heap;

		// OK, now let's calculate some averages for the local flock.
		var avgx = 0;
		var avgy = 0;
		var avgdx = 0;
		var avgdy = 0;
		var avgfx = 0;
		var avgfy = 0;
		var avgnm = blfn[item.type];
		var avgn = 0;
		for(j=0;j<avgnm&&j<closest.length;j++) {
			avgn++;
			var i2 = items[closest[j]];
			avgx += i2.x;
			avgy += i2.y;
			avgdx += i2.dx;
			avgdy += i2.dy;
			avgfx += i2.x + (i2.dx * bspeed[i2.type] * btfft[item.type]);
			avgfy += i2.y + (i2.dy * bspeed[i2.type] * btfft[item.type]);
		}
		// If there's anything like a local flock...
		if(avgn > 0) {
			avgx /= avgn;
			avgy /= avgn;
			avgdx /= avgn;
			avgdy /= avgn;
			avgfx /= avgn;
			avgfy /= avgn;
			
			// Turn towards local flock
			var tfm = btfm[item.type];
			var tfmt = btfmt[item.type];
			
			var head = r2d * Math.atan2(avgx - item.x, item.y - avgy);
			var dista = Math.sqrt(((item.x - avgx)*(item.x - avgx))+((item.y - avgy)*(item.y - avgy)));
			var hd = head - item.dir;
			if(hd > 180) hd -= 360;
			if(hd < -180) hd += 360;
			if(hd < 0 && hd > -180) {
				turn -= Math.min(-hd * tfm, tfmt);
			} else if(hd >=0 && hd < 180) {
				turn += Math.min(hd * tfm, tfmt);
			}

			// Turn towards local flock forecast
			var tffm = btffm[item.type];
			var tffmt = btffmt[item.type];

			head = r2d * Math.atan2(avgfx - item.x, item.y - avgfy);
			dista = Math.sqrt(((item.x - avgx)*(item.x - avgx))+((item.y - avgy)*(item.y - avgy)));
			hd = head - item.dir;
			if(hd > 180) hd -= 360;
			if(hd < -180) hd += 360;
			if(hd < 0 && hd > -180) {
				turn -= Math.min(-hd * tffm, tffmt);
			} else if(hd >=0 && hd < 180) {
				turn += Math.min(hd * tffm, tffmt);
			}
			
			// Match heading with local flock
			var mhm = bmhm[item.type];
			var mhmt = bmhmt[item.type];
			
			var avghead = r2d*Math.atan2(avgdx, -avgdy);
			hd = item.dir + turn - avghead;
			if(hd > 180) hd -= 360;
			if(hd < -180) hd += 360;
			if(hd < 0 && hd > -180) {
				turn += Math.min(-hd * mhm, mhmt);
			} else if(hd > 0 && hd < 180) {
				turn -= Math.min(hd * mhm, mhmt);
			}
		}
		
		// Edge avoidance
		
		var edged = bedged[item.type];
		var edgem = bedgem[item.type];
		var edgemt = bedgemt[item.type];

		if(item.y < edged) {
			if(item.dir > 0 && item.dir < 145) {
				turn += Math.min(edgemt,edgem * (edged - item.y));
			} else if(item.dir <= 0 && item.dir > -145) {
				turn += -Math.min(edgemt,edgem * (edged - item.y));
			}
		} else if(item.y > (canvas.height - edged)) {
			if(item.dir > -180 && item.dir < -45) {
				turn += Math.min(edgemt,edgem * (item.y - canvas.height + edged)); 
			} else if(item.dir <= 180 && item.dir > 45) {
				turn += -Math.min(edgemt,edgem * (item.y - canvas.height + edged));
			}		
		}
		if(item.x < edged) {
			if((item.dir < -90) || (item.dir > 145)) {
				turn += -Math.min(edgemt,edgem * (edged - item.x));				
			} else if(item.dir >= -90 && item.dir < 45) {
				turn += Math.min(edgemt,edgem * (edged - item.x));				
			}
		} else if(item.x > (canvas.width - edged)) {
			if((item.dir > 90) || (item.dir < -145)) {
				turn += Math.min(edgemt, edgem * (item.x - canvas.width + edged));
			} else if(item.dir <= 90 && item.dir > -45) {
				turn += -Math.min(edgemt, edgem * (item.x - canvas.width + edged));
			}
		}

		// Avoid collisions with other boids
		
		cold = bcold[item.type];
		colm = bcolm[item.type];
		colmt = bcolmt[item.type];
		
		for(j=0;j<items.length;j++) {
			if(i != j) {
				var i2 = items[j];
				var dist = dl[j];
				if(dist < cold) {
					var head = r2d * Math.atan2(i2.x - item.x, item.y - i2.y);
					var hd = head - (item.dir + turn);
					if(hd > 180) hd -= 360;
					if(hd < -180) hd += 360;
					if(hd < 0 && hd > -180) {
						turn += Math.min(colmt, colm * (cold - dist));
					} else if(hd >=0 && hd < 180) {
						turn -= Math.min(colmt, colm * (cold - dist));
					}
				}
			}
		}

		if(brtg[item.type] != 0) turn += rnd_gauss() * brtg[item.type];
		if(brtc[item.type] != 0) turn += rnd_cauchy() * brtc[item.type];
		turn = Math.min(bmturn[item.type], Math.max(-bmturn[item.type], turn));
		
		item.ndir = item.dir + turn;
	}
	
	// Update positions
	for(i=0;i<items.length;i++) {
		item = items[i];
		item.dir = item.ndir;
		var xv = Math.sin(item.dir * d2r) * bspeed[item.type];
		var yv = -Math.cos(item.dir * d2r) * bspeed[item.type];
		item.x += xv;
		item.y += yv;
		
		if(item.x < 0) item.x = 0;
		if(item.y < 0) item.y = 0;
		if(item.x >= canvas.width) item.x = canvas.width - 1;
		if(item.y >= canvas.height) item.y = canvas.height - 1;
		while(item.dir > 180) item.dir -= 360;
		while(item.dir < -180) item.dir += 360;
		//console.log(item.dir);
	}
	nframes++;
	if(nframes % 10 == 0) {
		var e = document.getElementById("status");
		e.innerHTML = "Frames: " + nframes;
		e.innerHTML += " Calculation time per frame: " + (Date.now()-tt) + " ms";
	}
}

function drawpointy(x, y, dir, angle, size, c) {
	ctx.fillStyle = c;
	ctx.strokeStyle = c;
	ctx.beginPath();

	ctx.moveTo(x+(Math.sin(dir*d2r)*size),y-(Math.cos(dir*d2r)*size));
	ctx.lineTo(x+(Math.sin((dir+angle)*d2r)*size),y-(Math.cos((dir+angle)*d2r)*size));
	ctx.lineTo(x+(Math.sin((dir-angle)*d2r)*size),y-(Math.cos((dir-angle)*d2r)*size));
	ctx.lineTo(x+(Math.sin(dir*d2r)*size),y-(Math.cos(dir*d2r)*size));
	
	ctx.fill();
	ctx.stroke();
}

// Draw a frame
function render() {
	// Background
	ctx.fillStyle = '#000';
	ctx.fillRect(0, 0, canvas.width, canvas.height);
	// Per item
	for(var i=0;i<items.length;i++) {
		var item = items[i];
		drawpointy(item.x, item.y, item.dir, 150, 5, bc[item.type]);
	}	
}

// Do a frame
function run() {
	//update();
	update();
	render();
}

// Set up
function reset() {
	readform();
	presetup();
	setup();
	endsetup();
}

reset();

setInterval(run, 30);