const evBookingFirebaseConfig = {
  apiKey: "AIzaSyABKehBoLgMYw20B4cwo6v8xVvq5zQi53M",
  authDomain: "esp-stream-test.firebaseapp.com",
  databaseURL: "https://esp-stream-test-default-rtdb.europe-west1.firebasedatabase.app",
  projectId: "esp-stream-test",
  storageBucket: "esp-stream-test.appspot.com",
  appId: "1:238762683803:web:2b310f1bc33d861cb2053e",
};

if (!firebase.apps.length) {
  firebase.initializeApp(evBookingFirebaseConfig);
}

window.evBookingDatabase = firebase.database();