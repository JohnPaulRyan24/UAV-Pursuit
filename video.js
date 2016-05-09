"use strict";
var bebop = require("../."),
    fs = require("fs");

var output = fs.createWriteStream("./video.h264"),
    drone = bebop.createClient();
    




function shimmy(){
   var inp = fs.readFileSync('file.txt').toString();
    if(inp==="0"){
        drone.stop();
    }else if(inp==="1"){
        setTimeout(function(){
            drone.clockwise(20);
    }, 400);
    setTimeout(function(){
            drone.stop();
    }, 800);
fs.writeFile("file.txt", "0");
    }else{
        drone.land();
    }
}

drone.connect(function() {
    
    drone.takeOff();
    setInterval(shimmy, 1000);
    setTimeout(function(){
            var video = drone.getVideoStream();video.pipe(output);
                drone.MediaStreaming.videoEnable(1);
            console.log('Start video');},3000);

});
