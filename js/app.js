/*
 * Copyright 2016 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the
 * License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * FirebaseUI initialization to be used in a Single Page application context.
 */

/**
* Custom selector "JQuery style", but in plain "Vanilla JS"
*/

const imageOn  = '<td><img src="img/light-on.svg" alt="light on" width="32"></img></td>';
const imageOff = '<td><img src="img/light-off.svg" alt="light off" width="32"></img></td>';

var $ = function(el) {
	return document.getElementById(el);
}

/**
 * @return {!Object} The FirebaseUI config.
 */
function getUiConfig() {
  return {
    'callbacks': {
      // Called when the user has been successfully signed in.
      'signInSuccessWithAuthResult': function(authResult, redirectUrl) {
        if (authResult.user) {
          handleSignedInUser(authResult.user);
        }
        // Do not redirect.
        return false;
      }
    },
    // Opens IDP Providers sign-in flow in a popup.
    'signInFlow': 'popup',
    'signInOptions': [
      // TODO(developer): Remove the providers you don't need for your app.
      {
        provider: firebase.auth.GoogleAuthProvider.PROVIDER_ID,
        // Required to enable ID token credentials for this provider.
        clientId: CLIENT_ID
      },

      {
        provider: firebase.auth.EmailAuthProvider.PROVIDER_ID,
        // Whether the display name should be displayed in Sign Up page.
        requireDisplayName: true,
        signInMethod: 'password',
        disableSignUp: {
          status: getDisableSignUpStatus()
        }
      }
    ],
    // Terms of service url.
    'tosUrl': 'https://www.google.com',
    // Privacy policy url.
    'privacyPolicyUrl': 'https://www.google.com',
    'credentialHelper': CLIENT_ID && CLIENT_ID != 'YOUR_OAUTH_CLIENT_ID' ?
        firebaseui.auth.CredentialHelper.GOOGLE_YOLO :
        firebaseui.auth.CredentialHelper.NONE,
    'adminRestrictedOperation': {
      status: getAdminRestrictedOperationStatus()
    }
  };
}

// Initialize the FirebaseUI Widget using Firebase.
var ui = new firebaseui.auth.AuthUI(firebase.auth());
// Disable auto-sign in.
ui.disableAutoSignIn();


/**
 * Displays the UI for a signed in user.
 * @param {!firebase.User} user
 */
var handleSignedInUser = function(user) {
  $('user-signed-in').style.display = 'block';
  $('user-signed-out').style.display = 'none';
  $('name').textContent = user.displayName + ' (';
  $('email').textContent = user.email + ')';

  // if (user.photoURL) {
  //   var photoURL = user.photoURL;
  //   // Append size to the photo URL for Google hosted images to avoid requesting
  //   // the image with its original resolution (using more bandwidth than needed)
  //   if ((photoURL.indexOf('googleusercontent.com') != -1) ||(photoURL.indexOf('ggpht.com') != -1)) {
	// 	photoURL = photoURL + '?sz=32' ;//+ $('photo').clientHeight;
  //   }
  //   $('photo').src = photoURL;
  //   $('photo').style.display = 'block';
  // }
  // else {
  //   $('photo').style.display = 'none';
  // }
};


/**
 * Displays the UI for a signed out user.
 */
var handleSignedOutUser = function() {
  document.getElementById('user-signed-in').style.display = 'none';
  document.getElementById('user-signed-out').style.display = 'block';
  ui.start('#firebaseui-container', getUiConfig());
};

// Listen to change in auth state so it displays the correct UI for when the user is signed in or not.
firebase.auth().onAuthStateChanged(function(user) {
  document.getElementById('loading').style.display = 'none';
  document.getElementById('loaded').style.display = 'block';
  user ? handleSignedInUser(user) : handleSignedOutUser();
});


/**
 * Deletes the user's account.
 */
var deleteAccount = function() {
  if ( confirm("User credentials will be deleted. Are you sure?")) {
    firebase.auth().currentUser.delete().catch(function(error) {
		if (error.code == 'auth/requires-recent-login') {
		  // The user's credential is too old. She needs to sign in again.
		  firebase.auth().signOut().then(function() {
			// The timeout allows the message to be displayed after the UI has changed to the signed out state.
			setTimeout(function() {
			  alert('Please sign in again to delete your account.');
			}, 1);
		  });
		}
	});
  }
};


/**
* Read some data from database
*/
function readGpioList() {
	const dbRef = firebase.database().ref();
	dbRef.child("esp-stream").child("gpios").get().then((snapshot) => {
	  if (snapshot.exists()) {
		  let gpios = snapshot.val();
		  console.log(gpios);
		  createGpios(gpios);
	  }
    else {
		  console.log("No data available");
	  }
	}).catch((error) => {
	  console.error(error);
	});
}

/**
* Send data "cmds" to database. ESP, will receive a related "stream" event
*/
function sendCommand(cmd, pin, level) {
  // Il contenuto del messaggio da inviare
  var postData = {
    cmd: cmd,
    pin: pin,
    level: !level
  };

  var updates = {};
  updates['/esp-stream/cmds'] = postData;
  console.log(updates);
  return firebase.database().ref().update(updates);
}

/**
* Iterate to the gpio list passaed as parameter anc create DOMs dinamically
*/

function updateGpios(elems) {

  const list = document.querySelector('#gpio-list')
  list.innerHTML = "";

	elems.forEach(elem => {
    var imgButton;

    // Show tootip if pin is inverted
    var tootipClass = elem.invert ? 'tooltiptext' : 'hidden';

    // Prepare button HTML with parameter anc tooltip class if needed
    var offButton = `<td class="tooltip"><button class="button is-medium" onclick="sendCommand(\'writeOut\', ${elem.pin}, ${elem.level});">` +
                    `<span class="icon is-medium"><img src="img/light-on.svg" width="32"></span><span style="font-size: 0.7em">Turn<br>OFF</span></button>` +
                    `<span class="${tootipClass}">PIN inverted.<br> Output ON (active) whith LOW level.</span></td>`;

    var onButton  =  `<td class="tooltip"><button class="button is-medium" onclick="sendCommand(\'writeOut\', ${elem.pin}, ${elem.level});">` +
                     `<span class="icon is-medium"><img src="img/light-off.svg" width="32"></span><span style="font-size: 0.7em">Turn<br>ON</span></button>` +
                     `<span class="${tootipClass}">PIN inverted.<br> Output ON (active) whith LOW level.</span></td>`;

    // Create a simple image for inputs, and a button + image for outputs accoring to pin level
    if (elem.level) {
      if(elem.type == 'output')
        imgButton = elem.invert ? onButton : offButton;
      else
        imgButton = imageOn;
    }
    else {
      if(elem.type == 'output')
        imgButton = elem.invert ? offButton : onButton;
      else
        imgButton = imageOff;
    }

    // Create a single row with all columns
    var row = document.createElement('tr');
	  row.innerHTML  = '<td>' + elem.label + '</td>';
    row.innerHTML += '<td>' + elem.pin + '</td>';
    row.innerHTML += '<td>' + elem.type + '</td>';
    row.innerHTML += imgButton;

    // Add row to list
    list.appendChild(row);
  })
}


/**
 * Initializes the app.
 */
var initApp = function() {

  document.getElementById('sign-out').addEventListener('click', function() {
    firebase.auth().signOut();
  });

  document.getElementById('delete-account').addEventListener(
  'click', function() {
	deleteAccount();
  });

 // readGpioList();

  // Add handler for data value events (usually updated gpios state list)
  var dataListener = firebase.database().ref('/esp-stream/gpios');
  dataListener.on('value', (snapshot) => {
    const data = snapshot.val();
    console.log(data);
    updateGpios(data);
  });


};

window.addEventListener('load', initApp);
