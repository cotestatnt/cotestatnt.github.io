/*
 *This program is free software: you can redistribute it and/or modify
 *it under the terms of the GNU General Public License as published by
 *the Free Software Foundation, either version 3 of the License, or
 *(at your option) any later version.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *You should have received a copy of the GNU General Public License
 *along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 
(function(ext) {

  var PIN_MODE = 0xF4,
    REPORT_DIGITAL = 0xD0,
    REPORT_ANALOG = 0xC0,
    DIGITAL_MESSAGE = 0x90,
    START_SYSEX = 0xF0,
    END_SYSEX = 0xF7,
    QUERY_FIRMWARE = 0x79,
    REPORT_VERSION = 0xF9,
    ANALOG_MESSAGE = 0xE0,
    ANALOG_MAPPING_QUERY = 0x69,
    ANALOG_MAPPING_RESPONSE = 0x6A,
    CAPABILITY_QUERY = 0x6B,
    CAPABILITY_RESPONSE = 0x6C;
	SERIAL_MESSAGE = 0x60;
	// DFPlayer
	PLAY_SONG = 0x03;
	NEXT = 0x01;
	PREV = 0x02;
	PLAY = 0x0D;
	PAUSE = 0x0E;
	VOLUME = 0x06;
	
	
  var INPUT = 0x00,
    OUTPUT = 0x01,
    ANALOG = 0x02,
    PWM = 0x03,
	SOFT_PWM = 0xF3,
    SERVO = 0x04,
    SHIFT = 0x05,
    I2C = 0x06,
    ONEWIRE = 0x07,
    STEPPER = 0x08,
    ENCODER = 0x09,
    SERIAL = 0x0A,
    PULLUP = 0x0B,
    IGNORE = 0x7F,
    TOTAL_PIN_MODES = 13;

  var LOW = 0,
    HIGH = 1;

  var MAX_DATA_BYTES = 4096;
  var MAX_PINS = 128;

  var parsingSysex = false,
    waitForData = 0,
    executeMultiByteCommand = 0,
    multiByteChannel = 0,
    sysexBytesRead = 0,
    storedInputData = new Uint8Array(MAX_DATA_BYTES);

  var digitalOutputData = new Uint8Array(16),
    digitalInputData = new Uint8Array(16),
    analogInputData = new Uint16Array(16);

  var analogChannel = new Uint8Array(MAX_PINS);
  var pinModes = [];
  for (var i = 0; i < TOTAL_PIN_MODES; i++) pinModes[i] = [];

  var majorVersion = 0,
    minorVersion = 0;

  var connected = false;
  var notifyConnection = true;
  var device = null;
  var inputData = null;

  // TEMPORARY WORKAROUND
  // Since _deviceRemoved is not used with Serial devices
  // ping device regularly to check connection
  var pinging = false;
  var pingCount = 0;
  var pinger = null;

  var hwList = new HWList();

  function HWList() {
    this.devices = [];

    this.add = function(dev, pin) {
      var device = this.search(dev);      
      if (!device) {
        device = {name: dev, pin: pin, val: 0};
        this.devices.push(device);
      } else {
        device.pin = pin;
        device.val = 0;
      }
    };

    this.search = function(dev) {
      for (var i=0; i<this.devices.length; i++) {
        if (this.devices[i].name === dev)
          return this.devices[i];
      }
      return null;
    };
  }

  function init() {

    for (var i = 0; i < 16; i++) {
      var output = new Uint8Array([REPORT_DIGITAL | i, 0x01]);
      device.send(output.buffer);
    }

    queryCapabilities();

    // TEMPORARY WORKAROUND
    // Since _deviceRemoved is not used with Serial devices
    // ping device regularly to check connection
    pinger = setInterval(function() {
      if (pinging) {
        if (++pingCount > 6) {
          clearInterval(pinger);
          pinger = null;
          connected = false;
          if (device) device.close();
          device = null;
          return;
        }
      } else {
        if (!device) {
          clearInterval(pinger);
          pinger = null;
          return;
        }
        queryFirmware();
        pinging = true;
      }
    }, 100);
  }

  function hasCapability(pin, mode) {
    if (pinModes[mode].indexOf(pin) > -1)
      return true;
    else
      return false;
  }

  function queryFirmware() {
    var output = new Uint8Array([START_SYSEX, QUERY_FIRMWARE, END_SYSEX]);
    device.send(output.buffer);
  }

  function queryCapabilities() {
    console.log('Querying ' + device.id + ' capabilities');
    var msg = new Uint8Array([
        START_SYSEX, CAPABILITY_QUERY, END_SYSEX]);
    device.send(msg.buffer);
  }

  function queryAnalogMapping() {
    console.log('Querying ' + device.id + ' analog mapping');
    var msg = new Uint8Array([
        START_SYSEX, ANALOG_MAPPING_QUERY, END_SYSEX]);
    device.send(msg.buffer);
  }
  

  function setDigitalInputOutput(){
	// Digital Input
	hwList.add(menus[lang]['buttons'][0], 16); 
	hwList.add(menus[lang]['buttons'][1], 17); 
	hwList.add(menus[lang]['buttons'][2], 18); 
	hwList.add(menus[lang]['buttons'][3], 19); 	
	
	//RGB1 (real pin are defined inside Arduino firmware (9, 10, 11)
	hwList.add(menus[lang]['leds'][0], 9); 
	hwList.add(menus[lang]['leds'][1], 10); 
	hwList.add(menus[lang]['leds'][2], 11);
	// RGB2
	hwList.add(menus[lang]['leds'][3], 3); 
	hwList.add(menus[lang]['leds'][4], 5); 
	hwList.add(menus[lang]['leds'][5], 6); 
	// Analog Input
	hwList.add(menus[lang]['hwIn'][0], 0); 
	hwList.add(menus[lang]['hwIn'][1], 1); 	
	// Servo motor
	hwList.add(menus[lang]['servos'][0], 7); 
	hwList.add(menus[lang]['servos'][1], 4); 
	
	hwList.add(menus[lang]['digitalIn'][0], 2); 
	
	
	pinMode(hwList.devices[0].pin, PULLUP);	
	pinMode(hwList.devices[1].pin, PULLUP);	
	pinMode(hwList.devices[2].pin, PULLUP);	
	pinMode(hwList.devices[3].pin, PULLUP);	
	
	pinMode(hwList.devices[4].pin, SOFT_PWM);
	pinMode(hwList.devices[5].pin, SOFT_PWM);
	pinMode(hwList.devices[6].pin, SOFT_PWM);	
	
	pinMode(hwList.devices[7].pin, PWM);
	pinMode(hwList.devices[8].pin, PWM);
	pinMode(hwList.devices[9].pin, PWM);	
	
	pinMode(hwList.devices[10].pin, ANALOG);
	pinMode(hwList.devices[11].pin, ANALOG);
	
	pinMode(hwList.devices[12].pin, SERVO);
	pinMode(hwList.devices[13].pin, SERVO);
	
	pinMode(hwList.devices[14].pin, INPUT);	
	
	rotateServo(12, 5);
	rotateServo(13, 5);	
	rotateServo(12, 175);
	rotateServo(13, 175);	
	rotateServo(12, 90);
	rotateServo(13, 90);
	
	console.log("Digital Input/Output configured");  
  }
  function setDigitalInputs(portNum, portData) {
    digitalInputData[portNum] = portData;
  }

  function setAnalogInput(pin, val) {
    analogInputData[pin] = val;
  }

  function setVersion(major, minor) {
    majorVersion = major;
    minorVersion = minor;
  }

  function processSysexMessage() {
    switch(storedInputData[0]) {
      case CAPABILITY_RESPONSE:
        for (var i = 1, pin = 0; pin < MAX_PINS; pin++) {
          while (storedInputData[i++] != 0x7F) {
            pinModes[storedInputData[i-1]].push(pin);
            i++; //Skip mode resolution
          }
          if (i == sysexBytesRead) break;
        }
        queryAnalogMapping();
		setDigitalInputOutput();		
        break;
      case ANALOG_MAPPING_RESPONSE:
        for (var pin = 0; pin < analogChannel.length; pin++)
          analogChannel[pin] = 127;
        for (var i = 1; i < sysexBytesRead; i++)
          analogChannel[i-1] = storedInputData[i];
        for (var pin = 0; pin < analogChannel.length; pin++) {
          if (analogChannel[pin] != 127) {
            var out = new Uint8Array([
                REPORT_ANALOG | analogChannel[pin], 0x01]);
            device.send(out.buffer);
          }
        }
        notifyConnection = true;
        setTimeout(function() {
          notifyConnection = false;
        }, 100);
        break;
      case QUERY_FIRMWARE:
        if (!connected) {
          clearInterval(poller);
          poller = null;
          clearTimeout(watchdog);
          watchdog = null;
          connected = true;
          setTimeout(init, 200);
        }
        pinging = false;
        pingCount = 0;
        break;
    }
  }

  function processInput(inputData) {
    for (var i=0; i < inputData.length; i++) {
      if (parsingSysex) {
        if (inputData[i] == END_SYSEX) {
          parsingSysex = false;
          processSysexMessage();
        } else {
          storedInputData[sysexBytesRead++] = inputData[i];
        }
      } else if (waitForData > 0 && inputData[i] < 0x80) {
        storedInputData[--waitForData] = inputData[i];
        if (executeMultiByteCommand !== 0 && waitForData === 0) {
          switch(executeMultiByteCommand) {
            case DIGITAL_MESSAGE:
              setDigitalInputs(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
              break;
            case ANALOG_MESSAGE:
              setAnalogInput(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
              break;
            case REPORT_VERSION:
              setVersion(storedInputData[1], storedInputData[0]);
              break;
          }
        }
      } else {
        if (inputData[i] < 0xF0) {
          command = inputData[i] & 0xF0;
          multiByteChannel = inputData[i] & 0x0F;
        } else {
          command = inputData[i];
        }
        switch(command) {
          case DIGITAL_MESSAGE:
          case ANALOG_MESSAGE:
          case REPORT_VERSION:
            waitForData = 2;
            executeMultiByteCommand = command;
            break;
          case START_SYSEX:
            parsingSysex = true;
            sysexBytesRead = 0;
            break;
		  default:
			//console.log('Received: ' + inputData);
			break;
        }
      }
    }
  }

   
  function pinMode(pin, mode) {
    var msg = new Uint8Array([PIN_MODE, pin, mode]);
    device.send(msg.buffer);
    //console.log('pinMode' + msg);
  }

  function analogRead(pin) {
    if (pin >= 0 && pin < pinModes[ANALOG].length) {
      return Math.round((analogInputData[pin] * 100) / 1023);
    } else {
      var valid = [];
      for (var i = 0; i < pinModes[ANALOG].length; i++)
        valid.push(i);
      console.log('ERROR: valid analog pins are ' + valid.join(', '));
      return;
    }
  }

  function digitalRead(pin) {
    if (!hasCapability(pin, INPUT)) {
      console.log('ERROR: valid input pins are ' + pinModes[INPUT].join(', '));
      return;
    }
	
    //pinMode(pin, INPUT);
	var val =  (digitalInputData[pin >> 3] >> (pin & 0x07)) & 0x01;
	console.log('digitalRead ' + pin + ' : ' + val);
    return val;
  }
  
  function digitalWrite(pin, val) {
    if (!hasCapability(pin, OUTPUT)) {
      console.log('ERROR: valid output pins are ' + pinModes[OUTPUT].join(', '));
      return;
    }
    var portNum = (pin >> 3) & 0x0F;
    if (val == LOW)
      digitalOutputData[portNum] &= ~(1 << (pin & 0x07));
    else
      digitalOutputData[portNum] |= (1 << (pin & 0x07));
    
	//pinMode(pin, OUTPUT);
    var msg = new Uint8Array([
        DIGITAL_MESSAGE | portNum,
        digitalOutputData[portNum] & 0x7F,
        digitalOutputData[portNum] >> 0x07]);
    device.send(msg.buffer);
	//console.log('digitalWrite ' + msg );
  }

  
  
  
  function analogWrite(pin, val) {
	
    if (pin == 9 | pin == 10 | pin == 11) {              
		console.log('Software PWM on pin ' + pin);		
    } 
	else if (hasCapability(pin, PWM)) {              
		console.log('Hardware PWM on pin ' + pin);
    } 
	else {
		console.log('ERROR: valid PWM pins are ' + pinModes[PWM].join(', '));
		return;	
	}
				
	if (val < 0) val = 0;
	else if (val > 100) val = 100;
	val = Math.round((val / 100) * 255);
	//pinMode(pin, PWM);
	var msg = new Uint8Array([
		ANALOG_MESSAGE | (pin & 0x0F),
		val & 0x7F,
		val >> 7]);	
	
    device.send(msg.buffer);
	console.log('analogWrite ' + msg );
  }

  
  
  
  function rotateServo(pin, deg, delay) {	
    if (!hasCapability(pin, SERVO)) {
      console.log('ERROR: valid servo pins are ' + pinModes[SERVO].join(', '));
      return;
    }
    pinMode(pin, SERVO);
    var msg = new Uint8Array([
        ANALOG_MESSAGE | (pin & 0x0F),
        deg & 0x7F,
        deg >> 0x07]);
    device.send(msg.buffer);
	console.log('rotateServo ' + msg );	
	setTimeout(stopServo, delay);
	function stopServo() {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}
  }
  
  
  function mapValues(val, aMin, aMax, bMin, bMax) {
    var output = (((bMax - bMin) * (val - aMin)) / (aMax - aMin)) + bMin;
    return Math.round(output);
  };
  
  
  ext.whenConnected = function() {	 
    if (notifyConnection) return true;
    return false;
  };
	
	
  
  // CUSTOM external 
  ext.play = function(){
	console.log('Play music');
    var msg = new Uint8Array([START_SYSEX, SERIAL_MESSAGE, PLAY, 0x00, END_SYSEX]);    
	device.send(msg.buffer);
	console.log(msg);	  
  };
  
  ext.pause = function(){
	console.log('Stop music');
    var msg = new Uint8Array([START_SYSEX, SERIAL_MESSAGE, PAUSE, 0x00, END_SYSEX]);    
	device.send(msg.buffer);
	console.log(msg);	  
  };
  
  ext.next = function(){
	console.log('Play music');
    var msg = new Uint8Array([START_SYSEX, SERIAL_MESSAGE, NEXT, 0x00, END_SYSEX]);    
	device.send(msg.buffer);
	console.log(msg);	  
  };
  
  ext.prev = function(){
	console.log('Stop music');
    var msg = new Uint8Array([START_SYSEX, SERIAL_MESSAGE, PREV, 0x00, END_SYSEX]);    
	device.send(msg.buffer);
	console.log(msg);	  
  };
 
  
  ext.playSong = function(song){
	console.log('Play song ' + song);  
    var msg = new Uint8Array([START_SYSEX, SERIAL_MESSAGE, PLAY_SONG, song, END_SYSEX]);   
	device.send(msg.buffer);
	console.log(msg);	  
  };    
  
  ext.volume = function(volume){
	var DSPVolume = mapValues(volume, 0, 100, 0, 32);
	console.log('Set volume ' + DSPVolume);  
    var msg = new Uint8Array([START_SYSEX, SERIAL_MESSAGE, VOLUME, DSPVolume, END_SYSEX]);   
	device.send(msg.buffer);
	console.log(msg);	  
  };  
  
  ext.digitalRead = function(pin) {
	console.log('ext digitalRead ' + pin );
    return digitalRead(pin);
  };
  
  ext.digitalWrite = function(pin, val) {
	console.log('ext digitalWrite ' + pin );
    if (val == menus[lang]['outputs'][0])
      digitalWrite(pin, HIGH);
    else if (val == menus[lang]['outputs'][1])
      digitalWrite(pin, LOW);    
  };
  
  
  ext.digitalLED = function(led, val) {
    var hw = hwList.search(led);   
	console.log('ext.digitalLED ' + hw.pin +' ' + val );		
    if (!hw) return;
    if (val == menus[lang]['outputs'][0]) {
      digitalWrite(hw.pin, HIGH);
      hw.val = 255;
    } else {
      digitalWrite(hw.pin, LOW);
      hw.val = 0;
    }    
  };
  
  ext.isButtonPressed = function(btn) {
    var hw = hwList.search(btn);	
	console.log('ext.isButtonPressed ' + digitalRead(hw.pin));	
    if (!hw) return;		
    return !digitalRead(hw.pin);
  };
  
  
  ext.isPirActive = function(pir) {
    var hw = hwList.search(pir);	
	console.log('ext.isPirActive ' + digitalRead(hw.pin));	
    if (!hw) return;		
    return digitalRead(hw.pin);
  };

  ext.analogRead = function(pin) {	
    return analogRead(pin);
  };

  ext.analogWrite = function(pin, val) {
    analogWrite(pin, val);
  };


  ext.whenAnalogRead = function(pin, op, val) {
    if (pin >= 0 && pin < pinModes[ANALOG].length) {
      if (op == '>')
        return analogRead(pin) > val;
      else if (op == '<')
        return analogRead(pin) < val;
      else if (op == '=')
        return analogRead(pin) == val;
      else
        return false;
    }
  };

  ext.whenDigitalRead = function(pin, val) {
    if (hasCapability(pin, INPUT)) {
      if (val == menus[lang]['outputs'][0])
        return digitalRead(pin);
      else if (val == menus[lang]['outputs'][1])
        return digitalRead(pin) === false;
    }
  };

  ext.connectHW = function(hw, pin) {
    hwList.add(hw, pin);    
  };

  ext.rotateServo = function(servo, deg) {	
    var hw = hwList.search(servo);	
    if (!hw) return;
    if (deg < 0) deg = 0;
    else if (deg > 180) deg = 180;
	
	var delay = 4*Math.abs(deg - hw.val)+500;
    rotateServo(hw.pin, deg, delay);
    hw.val = deg;	
  };

  ext.changeServo = function(servo, change) {	
    var hw = hwList.search(servo);
    if (!hw) return;
    var deg = hw.val + change;
    if (deg < 0) deg = 0;
    else if (deg > 180) deg = 180;
	var delay = 4*Math.abs(deg - hw.val);
    rotateServo(hw.pin, deg, delay);
    hw.val = deg;	
  };

  ext.setLED = function(led, val) {
    var hw = hwList.search(led);
    if (!hw) return;
    analogWrite(hw.pin, val);
    hw.val = val;
  };

  ext.changeLED = function(led, val) {
    var hw = hwList.search(led);
    if (!hw) return;
    var b = hw.val + val;
    if (b < 0) b = 0;
    else if (b > 100) b = 100;
    analogWrite(hw.pin, b);
    hw.val = b;
  };
  
  
  ext.setRGB = function(RGBs, color) {
	//			  Bianco	Nero	  Rosso		Verde	  Blue		Arancio   Violetto  Azzurro, Acquamarina
    var colors = [0xFFFFFF, 0x000000, 0xFF0000, 0x00FF00, 0x0000FF, 0xFF9600, 0x9600FF, 0x0096FF, 0x00FF96];
    console.log('ext.setRGB ' + color );
	var idx = menus[lang]['color_list'].indexOf(color);		
	
	var R = mapValues( 0x0000FF & colors[idx] >> 16, 0, 255, 0, 100);
	var G = mapValues( 0x0000FF & colors[idx] >> 8, 0, 255, 0, 100);
	var B = mapValues( 0x0000FF & colors[idx], 0, 255, 0, 100);
	console.log('ext.setRGB ' + R + ' ' + G + ' ' + B );
	
	var rgb = menus[lang]['RGBs'].indexOf(RGBs)
	if(rgb == 0){
		// Soft PWM
		analogWrite(9, R);
		analogWrite(10, G);
		analogWrite(11, B);	
	}
    else {
		analogWrite(3, R);
		analogWrite(5, G);
		analogWrite(6, B);			
	}
  };
  
  
  
  
  ext.readInput = function(name) {
    var hw = hwList.search(name);		
    if (!hw) return;
	var anVal = analogRead(hw.pin); 
	console.log('ext.readInput ' + anVal );
    return anVal;
  };

  ext.whenButton = function(btn, state) {
    var hw = hwList.search(btn);
	console.log('ext.whenButton ' + hw );
    if (!hw) return;	
    if (state === menus[lang]['btnStates'][0])
      return !digitalRead(hw.pin);
    else if (state === menus[lang]['btnStates'][1])
      return digitalRead(hw.pin);
  };


  ext.whenInput = function(name, op, val) {
    var hw = hwList.search(name);	
    if (!hw) return;
	var res = false;		
    if (op == '>')
	  res = analogRead(hw.pin) > val;       
    else if (op == '<')
      res =  analogRead(hw.pin) < val;
    else if (op == '=')
      res =  analogRead(hw.pin) == val;
  
    console.log('ext.whenInput ' + hw );
    return res;
  };
 

  ext.mapValues = function(val, aMin, aMax, bMin, bMax) {
    var output = (((bMax - bMin) * (val - aMin)) / (aMax - aMin)) + bMin;
    return Math.round(output);
  };

  ext._getStatus = function() {
    if (!connected)
      return { status:1, msg:'Disconnected' };
    else
      return { status:2, msg:'Connected' };
  };

  ext._deviceRemoved = function(dev) {
    console.log('Device removed');
    // Not currently implemented with serial devices
  };

  var potentialDevices = [];
  ext._deviceConnected = function(dev) {
    potentialDevices.push(dev);
    if (!device)
      tryNextDevice();
  };

  var poller = null;
  var watchdog = null;
  function tryNextDevice() {
    device = potentialDevices.shift();
    if (!device) return;

    device.open({ stopBits: 0, bitRate: 57600, ctsFlowControl: 0 });
    console.log('Attempting connection with ' + device.id);
    device.set_receive_handler(function(data) {
      var inputData = new Uint8Array(data);
      processInput(inputData);
    });

    poller = setInterval(function() {
      queryFirmware();
    }, 1000);

    watchdog = setTimeout(function() {
      clearInterval(poller);
      poller = null;
      device.set_receive_handler(null);
      device.close();
      device = null;
      tryNextDevice();
    }, 5000);
  }

  ext._shutdown = function() {
    // TODO: Bring all pins down
    if (device) device.close();
    if (poller) clearInterval(poller);
    device = null;
  };

  // Check for GET param 'lang'
  var paramString = window.location.search.replace(/^\?|\/$/g, '');
  var vars = paramString.split("&");
  var lang = 'en';
  for (var i=0; i<vars.length; i++) {
    var pair = vars[i].split('=');
    if (pair.length > 1 && pair[0]=='lang')
      lang = pair[1];
  }

  var blocks = {
    it: [
      ['h', 'Quando Arduino è connesso', 'whenConnected'],	  
      //[' ', 'Connetti il %m.hwOut al pin %n', 'connectHW', 'LED Rosso A', 3],
      //[' ', 'Connetti il %m.hwIn ad analog %n', 'connectHW', 'Potenziometro', 0],
      ['-'],
      //[' ', 'Imposta %m.leds a %m.outputs', 'digitalLED', 'LED Rosso 1', 'acceso'],
      [' ', 'Imposta %m.leds a %n%', 'setLED', 'LED Rosso1', 100],
      [' ', 'Aumenta %m.leds di %n%', 'changeLED', 'LED Rosso1', 10],	  
	  [' ', 'Imposta %m.RGBs a %m.color_list', 'setRGB', 'RGB 1', 'Arancio'],
	  
	  
	  ['-'],
	  [' ', 'Avvia la musica', 'play'],
	  [' ', 'Ferma la musica', 'pause'],
	  [' ', 'Prossimo', 'next'],
	  [' ', 'Precedente', 'prev'],
	  [' ', 'Suona canzone %n', 'playSong', 1],
	  [' ', 'Imposta volume a %n', 'volume', 80],
	  
	    
      ['-'],
      [' ', 'Ruota %m.servos fino a %n gradi', 'rotateServo', 'Motore1', 90],
      [' ', 'Ruota %m.servos di %n gradi', 'changeServo', 'Motore1', 20],
      ['-'],
      ['h', 'Quando %m.buttons è %m.btnStates', 'whenButton', 'Pulsante A', 'Premuto'],
      ['b', '%m.buttons premuto?', 'isButtonPressed', 'Pulsante A'],
      ['b', '%m.digitalIn Occupato?', 'isPirActive', 'Sensore presenza'],
      ['-'],
      ['h', 'Quando %m.hwIn %m.ops %n%', 'whenInput', 'Potenziometro', '>', 50],
      ['r', 'Leggi %m.hwIn', 'readInput', 'Potenziometro'],
	  ['b', 'Se %m.hwIn %m.ops %n%', 'whenInput', 'Potenziometro', '>', 50],
      ['-']
	   /*
      [' ', 'Imposta pin %n a %m.outputs', 'digitalWrite', 1, 'acceso'],
      [' ', 'Porta pin %n al %n%', 'analogWrite', 3, 100],
      ['-'],
      ['h', 'Quando pin %n è %m.outputs', 'whenDigitalRead', 1, 'acceso'],
      ['b', 'pin %n acceso?', 'digitalRead', 1],
      ['-'],
      ['h', 'Quando segnale %n %m.ops %n%', 'whenAnalogRead', 1, '>', 50],
      ['r', 'Leggi segnale %n', 'analogRead', 0],
      ['-'],
      ['r', 'Porta %n da %n %n a %n %n', 'mapValues', 50, 0, 100, -240, 240]
	  */
    ]
  };

  var menus = {
    it: {
      buttons: ['Pulsante A', 'Pulsante B', 'Pulsante C', 'Pulsante D' ],
      btnStates: ['Premuto', 'Rilasciato'],
	  digitalIn: [ 'Sensore presenza'],
	  digitalStates: [ 'Occupato', 'Libero'],
      hwIn: ['Potenziometro', 'Sensore di luce'],
      hwOut: ['LED Rosso1', 'LED Verde1', 'LED Blu1', 'LED Rosso2', 'LED Verde2', 'LED Blu2', 'Motore1', 'Motore2'],
      leds: ['LED Rosso1', 'LED Verde1', 'LED Blu1', 'LED Rosso2', 'LED Verde2', 'LED Blu2'],
      outputs: ['acceso', 'spento'],
      ops: ['>', '=', '<'],
      servos: ['Motore1', 'Motore2'],
	  RGBs: ['RGB 1', 'RGB 2'],
	  color_list: ['Bianco', 'Nero', 'Rosso', 'Verde', 'Blu', 'Arancio', 'Violetto', 'Azzurro', 'Acquamarina']
    }
  };

  var descriptor = {
    blocks: blocks[lang],
    menus: menus[lang],
    url: 'http://khanning.github.io/scratch-arduino-extension'
  };

  ScratchExtensions.register('Arduino', descriptor, ext, {type:'serial'});

})({});