prompt :
create Arduino code for esp8266 synchronizing millis() timer of the device with other devices in the subnet by exchanging udp broadcast packets containing data useful for the task like global network packet sending frequency, own optimal time slot in global packet sending frequency, own timer value, wifi signal level, wifi noise level, calculated packet loss, estimated packet loss, estimated timer error . use convergence strategy utilizing packet delay times and variable global packet sending frequency, so each device gets own slot in the global packet sending frequency region of the whole network to which it converges to make system predictable. One slot should be reserved for pulsing built-in LED exactly at the global frequency and reporting diagnotics to serial console. use kalman filter to separate noise and signal . Allow use of multiple nodes with 4 defined as compile-time default. identfy nodes by their IP. make sure to construct valid kalman filter model including orthogonality and convergence of data and time offset sources like round trip time, include all data from the udp packets of all nodes to create the model. as it is complex task, try outlining strategy and assumptions first , then take extra step to define proper kalman filter model which includes not only sources of errors like round trip time, but also estimate of how much node is already synchronized and how much signal strength and noise should influence measurement noise. Then proceed to writing code.