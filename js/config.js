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
 
var config = {
  apiKey: "AIzaSyABKehBoLgMYw20B4cwo6v8xVvq5zQi53M",
  authDomain: "esp-stream-test.firebaseapp.com",
  databaseURL: "https://esp-stream-test-default-rtdb.europe-west1.firebasedatabase.app",
  projectId: "esp-stream-test",
  storageBucket: "esp-stream-test.appspot.com",  
  appId: "1:238762683803:web:2b310f1bc33d861cb2053e",
};
firebase.initializeApp(config);
var database = firebase.database();


// Google OAuth Client ID, needed to support One-tap sign-up.
// Set to null if One-tap sign-up is not supported.
var CLIENT_ID = null ;

