from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
#Linea 1 y 2 son librerias descargadas que se utilizaron en la programacion de sublime text
#Desde la linea 38 hasta la 64 son las coordenadas que se utilizan para poder dirigir el drone a las zonas especificas que elegimos

def arm_and_takeoff(TargetAltitude):
#Vehicle connection
	print("Executing takeoff")



	while not drone.is_armable:
		print("Vehicle is not armable, waiting....")
		time.sleep(1)

	print("ready to arm")
	drone.mode = VehicleMode("GUIDED")
	drone.armed = True
	while not drone.armed:
		print("Waiting for arming....")
		time.sleep(1)

	print("Ready for takooff, taking off...")
	drone.simple_takeoff(TargetAltitude)

	while True:
		Altitude = drone.location.global_relative_frame.alt
		print("altitude: ",Altitude)
		time.sleep(1)

		if Altitude >= TargetAltitude * 0.95:
			print("Altitude reached")
			break

#De la linea 6 hasta la 33 son utilizados como comandos para conectar el dron y poder elevarlo


#Vehicle connection
drone = connect('127.0.0.1:14551', wait_ready=True)
arm_and_takeoff(20)
#Coordenadas, altitud y velocidad
drone.airspeed = 10
b_point = LocationGlobalRelative(20.736416,-103.457405,20)
c_point = LocationGlobalRelative(20.736316,-103.456143,20)
d_point = LocationGlobalRelative(20.735453,-103.456197,20)
e_point = LocationGlobalRelative(20.735487,-103.457433,20)



print("Dirigiendose al punto b")
drone.simple_goto(b_point)
time.sleep(22)

print("Dirigiendose al punto c")
drone.simple_goto(c_point)
time.sleep(22)

print("Dirigiendose al punto d")
drone.simple_goto(d_point)
time.sleep(22)

print("Dirigiendose al punto e")
drone.simple_goto(e_point)
time.sleep(22)

print(drone.batery.level,"v")

drone.mode = VehicleMode("RTL")
