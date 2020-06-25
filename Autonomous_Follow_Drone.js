/* Leader Drone is the raspberry pi attached to a drone */
/* Follower Drone is Parrot AR Drone that will follow the GPS location of the Raspberry Pi */

const gpsd = require('node-gpsd'); // for the gps from the pi
var arDrone = require('ar-drone'); // for the parrot ar drone
const Gpio = require('pigpio').Gpio; // for the altitude from the pi

var droneClient = arDrone.createClient();
droneClient.disableEmergency();
droneClient.config('general:navdata_demo', 'FALSE'); // get back all data the copter can send
droneClient.config('general:navdata_options', 777060865); // turn on GPS module on ar drone

/*--------Begin Daemon for GPS module on Pi--------*/
var daemon = new gpsd.Daemon({
	device: '/dev/ttyACM0'
});
droneClient.disableEmergency();

droneClient.calibrate(1); //calibrate the drone's level sensor
droneClient.takeoff(); //ar drone takeoff


var lead_Lat, lead_Lon, follow_Lat, follow_Lon, theta, heading;
var start = new Date();
var foundbufferStart = new Date();
var calibrated = 0;
var lead_Alt, follow_Alt;

// The number of microseconds it takes sound to travel 1cm at 20 degrees celcius
const MICROSECDONDS_PER_CM = 1e6/34321;

const trigger = new Gpio(23, {mode: Gpio.OUTPUT});
const echo = new Gpio(24, {mode: Gpio.INPUT, alert: true});

trigger.digitalWrite(0); // Make sure trigger is low


function daemonInit() {
	console.log('GPSD Daemon Started');

	const LISTENER = new gpsd.Listener();

	LISTENER.on('TPV', function(tpv) {
		gpsData = tpv;
		console.log("Raspberry Pi Latitude: " + gpsData.lat + " Longitude: " + gpsData.lon); //display onto console Leader Drone gps latitude and longitude
		lead_Lat = gpsData.lat;
		lead_Lon = gpsData.lon;
	});

	droneClient.on('navdata', function(navdata) {
		try{

			//give drone some time to calibrate before taking off
			if((new Date() - start > 2000) && calibrated == 0){
				calibrated = 1;
				droneClient.calibrate(0);
			}

			//get latitude and longitude of Follower Drone
			follow_Lat = navdata.gps.latFuse;
			follow_Lon = navdata.gps.lonFuse;

			//get theta
			theta = calcHeading(follow_Lat,follow_Lon,lead_Lat,lead_Lon);

			//get heading of Follower Drone
			heading = (navdata.magneto.heading.fusionUnwrapped + 3600) % 360;

			//get distance between the follower and leader drone
			distance = calcDistance(follow_Lat,follow_Lon,lead_Lat,lead_Lon);

			//get altitude of follower drone
			follow_Alt = navdata.demo.altitudeMeters;

			//altitude difference between follower and leader drone
			Alt_Diff = lead_Alt - follow_Alt;

			//print everything out
			//console.log("latitude: " + follow_Lat + ", longitude: " + follow_Lon);
			console.log("theta: " + theta + "  heading:" + heading + " altitude: " + follow_Alt
						+ "  distance: " + distance + " altitude diff: " + (lead_Alt - follow_Alt) + "  battery: " + navdata.demo.batteryPercentage);

			//get elevation of follower drone
			elevation = navdata.gps.elevation;


			var end = new Date() - start;
			var foundbufferEnd = new Date() - foundbufferStart

			//if statement to allow drone to spin to correct heading
			if(end > 17000 && foundbufferEnd > 1000){ //'end' is to ensure drone does not start spinning until 17 seconds have passed
				if(Math.abs(Alt_Diff) < 1){ //if altitude difference is less than 1 meter adjust heading to point towards leader drone
					if((heading < theta - 3 || heading > theta + 3 || isNaN(theta) == true)){ //if the heading is not within 3 degrees of the correct heading
						// coding logic to ensure that drone is spinning the optimal direction
						if(heading <= 180){
							if(theta <= heading + 180 && theta > heading){
								droneClient.stop();
								droneClient.clockwise(0.25);
								console.log('spinning right')
							}
							else{
								droneClient.stop();
								droneClient.counterClockwise(0.25);
								console.log('spinning left')
							}
						}
						else{
							if(theta >= heading - 180 && theta < heading){
								droneClient.stop();
								droneClient.counterClockwise(0.25);
								console.log('spinning left')
							}
							else{
								droneClient.stop();
								droneClient.clockwise(0.25);
								console.log('spinning right')
							}
						}
					//if drone is within 3 degrees of the correct heading, then fly forwards
					}else{
						foundbufferStart = new Date();
						droneClient.stop();
						console.log('stopping');
						console.log("\n\n\nTHIS IS FINAL \nFINAL Theta: " + theta);
						console.log('FINAL Heading:' + heading + "\n\n\n");
						if(distance > 2){ //if the drone is greater than two meters away from the leader drone, than fly forwards, otherwise stop
				 	    droneClient.front(0.1);
				 	    console.log('FLYING FORWARD');
	          			}
					}
				}else{ // if altitude difference is greater than 1 meter, than adjust the altitude of the drone
					if(Alt_Diff > 1){ // if follower drone is lower than leader, then fly up
						droneClient.up(0.5);
						console.log('up');
					}else{ // otherwise fly down
						droneClient.down(0.5);
						console.log('down');
					}
				}
			}

		}catch(err){
		console.log(err.message);
		}
	});


	LISTENER.connect(function() {
		console.log('Connected');
		LISTENER.watch();
	});
}

function myFunc(){
	daemon.start(daemonInit);
}

//function to calculate the theta between the two GPS data of follower and leader drone
function calcHeading(fLat,fLon,lLat,lLon){
	//convert to radians
	F_Lat_Rad = fLat * 3.1415 / 180;
	F_Lon_Rad = fLon * 3.1415 / 180;

	L_Lat_Rad = lLat * 3.1415 / 180;
	L_Lon_Rad = lLon * 3.1415 / 180;

	//equation to measure the heading between the two coordinates
	theta = Math.atan2( Math.sin(L_Lon_Rad - F_Lon_Rad) * Math.cos(L_Lat_Rad),
				Math.cos(F_Lat_Rad) * Math.sin(L_Lat_Rad) - Math.sin(F_Lat_Rad) *
				Math.cos(L_Lat_Rad) * Math.cos(L_Lon_Rad - F_Lon_Rad) );

	thetaDegrees = ((theta * 180 / 3.1415) + 360) % 360;
	return thetaDegrees;
}

//function to calculate the distance from follower to leader drone
function calcDistance(fLat,fLon,lLat,lLon){
	var earthRadius = 6371000;

	//convert to radians
	F_Lat_Rad = fLat * 3.1415 / 180;
	L_Lat_Rad = lLat * 3.1415 / 180;

	var latDiff = (fLat - lLat) * 3.1415 / 180;
	var lonDiff = (fLon - lLon) * 3.1415 / 180;

	var haverSine = Math.sin((latDiff/2) * Math.sin(latDiff/2) + Math.cos(L_Lat_Rad) * Math.cos(F_Lat_Rad) * Math.sin(lonDiff/2) * Math.sin(lonDiff/2));

	//c (depends on HaverSine)
	var c = 2 * Math.atan2(Math.sqrt(haverSine), Math.sqrt(1-haverSine));

	//Complete distance formula
	var distance = earthRadius * c;
	return distance;
}

setTimeout(myFunc,1000,'funky');

const watchHCSR04 = () => {
  let startTick;

  echo.on('alert', (level, tick) => {
    if (level == 1) {
      startTick = tick;
    } else {
      const endTick = tick;
      const diff = (endTick >> 0) - (startTick >> 0); // Unsigned 32 bit arithmetic
      lead_Alt = diff / 2 / MICROSECDONDS_PER_CM / 100;
      console.log("Leader Drone Altitude:" + lead_Alt);
    }
  });
};

watchHCSR04();

// Trigger a distance measurement once per second
setInterval(() => {
  trigger.trigger(10, 1); // Set trigger high for 10 microseconds
}, 1000);
