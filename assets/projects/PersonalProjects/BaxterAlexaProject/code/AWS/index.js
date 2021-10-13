// Alexa SDK for OurWeather
// Copyright (c) 2014-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved. Use is subject to license terms.
// SwitchDoc Labs 2017 www.switchdoc.com
var PubNub = require("pubnub");

var JSONBaxterInfoMessage;

var pubnub = new PubNub({
    ssl: true,

    publish_key: "pub-c-a2209755-b405-4167-88d6-c34039971ef6",
    subscribe_key: "sub-c-2d3ae220-03c6-11e8-a55d-d67d19117359",
});
var baxterSubscribeChannel = 'Baxter_Status';
var alexaPublishChannel = 'Alexa_Data';


//context.callbackWaitsForEmptyEventLoop = true;


/**
 * App ID for the skill
 */
var APP_ID = 'amzn1.ask.skill.bdc949c7-dd6f-42db-a493-cdcddac2d6b1'; //replace with "amzn1.echo-sdk-ams.app.[your-unique-value-here]";

/**
 * The AlexaSkill prototype and helper functions
 */
var AlexaSkill = require('./AlexaSkill');

/**
 * Echobot is a child of AlexaSkill.
 * To read more about inheritance in JavaScript, see the link below.
 *
 * @see https://developer.mozilla.org/en-US/docs/Web/JavaScript/Introduction_to_Object-Oriented_JavaScript#Inheritance
 */



var BaxterAlexaControl = function() {



    AlexaSkill.call(this, APP_ID);

};



// Extend AlexaSkill
BaxterAlexaControl.prototype = Object.create(AlexaSkill.prototype);
BaxterAlexaControl.prototype.constructor = BaxterAlexaControl;


// Handle all our intents from Alexa Skill
BaxterAlexaControl.prototype.intentHandlers = {


    // register custom intent handlers
    
    
    // AboutBaxter intent
    AboutBaxter: function(intent, session, response) {
        var myText;
        console.log("in about");
        
        // Write out about Baxter
        myText = "Baxter is a 300-pound robot loctaed here at the Innovation Den, for the University of Idaho Coeur d'Alene Computer Science Program. Baxter is produced by Rethink Robotics, located in Boston, Massahcusetts. and provides a unique environment to work closely and safely without the need of plexiglass. Baxter can interact with the world and even make a cup of coffee.";
        response.tell(myText);
        return;
    },

    // GetBaxterStatus intent
    GetBaxterStatus: function(intent, session, response) {
        
        var first = pubnub.history({
            channel: baxterSubscribeChannel,     // This channel with give us necessary info about Baxter
            reverse: false,                     // Setting to true will traverse the time line in reverse starting with the oldest message first.
            count:1,                            // how many items to fetch
       
        }).then((response) => {
            console.log("response=", JSON.stringify(response));
            var obj = JSON.parse(JSON.stringify(response));
            
            var msgs = obj.messages;
            var msg = msgs[0];
            var parsedJSON = JSON.parse(JSON.stringify(msg.entry));
            
            JSONBaxterInfoMessage = parsedJSON;
            
        	return;         // I think this "return" doesn't exit the entire "function", just exits within the ".then" body of code, thus the code still executes the "Promise" (Javascript lack of knowledge)
       
       
        }).catch((error) => {
            console.log(error);
        });
        
         // Parse the promise to construct the request (Promise makes sure we got data)
         return Promise.all([first])
            .then(function(responses) {
                var myText;
                
                console.log("\n\nTest2\n\n");
                
                // These are filler data array access (until I decide on what my JSON message will consist of)
                if(JSONBaxterInfoMessage.Baxter_CurrentStatus == "Active") {         // 1 means good to run coffee program status
                   myText = "Baxter is ready to make coffee";
                }
                else if(JSONBaxterInfoMessage.Baxter_CurrentStatus == "Busy") {         // 2 means busy running other program status
                    myText = "Baxter is currently occupied with another task at the moment";
                }
                else {                              // Otherwose, we are off (value == 3, but will be assumed, for safety)
                    myText = "Baxter is not taking requests for coffee, Baxter's node for listening to Alexa is off";
                }
                
                response.tell(myText);
                
                return;
            });
    },

    // BaxterMakeCoffee intent
    BaxterMakeCoffee: function(intent, session, response) {
        
        var baxterMessage1 = {
            "Coffee_Request" : "Make Coffee" };
        
        pubnub.publish({ 
            channel   : alexaPublishChannel,
            message   : baxterMessage1,
            callback  : function(e) { 
                 console.log( "SUCCESS!", e ); 
                 //response.tell("Drone is flying");
                },
            error     : function(e) { 
                response.tell("Could not connect to Baxter, please try again");
                console.log( "FAILED! RETRY PUBLISH!", e ); }
        });   
        
        var first = pubnub.history({
            channel: baxterSubscribeChannel,     // This channel with give us necessary info about Baxter
            reverse: false,                     // Setting to true will traverse the time line in reverse starting with the oldest message first.
            count:1,                            // how many items to fetch
       
        }).then((response) => {
            console.log("response=", JSON.stringify(response));
            var obj = JSON.parse(JSON.stringify(response));
            
            var msgs = obj.messages;
            var msg = msgs[0];
            var parsedJSON = JSON.parse(JSON.stringify(msg.entry));
            
            JSONBaxterInfoMessage = parsedJSON;
            
        	return;         // I think this "return" doesn't exit the entire "function", just exits within the ".then" body of code, thus the code still executes the "Promise" (Javascript lack of knowledge)
       
       
        }).catch((error) => {
            console.log(error);
            process.exit(1);
        });
        
         // Parse the promise to construct the request (Promise makes sure we got data)
         return Promise.all([first])
            .then(function(responses) {
                var myText;
                
                console.log("JSON for Baxter_CurrenState: ");
                console.log(JSONBaxterInfoMessage.Baxter_CurrentStatus);
                console.log("CoffCommand: ");
                console.log(JSONBaxterInfoMessage.Baxter_CoffeeCommand);
                
                // Similar to Baxter Status, just so Alexa can update user
                if(JSONBaxterInfoMessage.Baxter_CurrentStatus == "Busy") {         // 2 means busy running other program status
                    myText = "Request denied, Baxter is currently occupied with another task at the moment";
                }
                else if(JSONBaxterInfoMessage.Baxter_CurrentStatus == "Off") {                              // Otherwose, we are off (value == 3, but will be assumed, for safety)
                    myText = "Request denied, Baxter is not taking requests for coffee, Baxter's node for listening to Alexa is off";
                }
                else {
                    myText = "Request accepted, Baxter is making coffee, please wait";
                }
                
                // Give our response to Alexa
                response.tell(myText);
                
                return;
                
            }).catch(function(error) {
            console.log(error);
        });
    },
    
     // BaxterJokeOne Intent
    BaxterJokeOne: function(intent, session, response) {
        var myText;
        console.log("in joke1");
        
         // Return number between 1 and 10
        var rand = Math.floor((Math.random() * 10) + 1);
        
        switch(rand) {
            case 1:
                myText = "Why are robots shy? Because they have hardware and software but no underware";
                break;
            case 2:
                myText = "Why was the robot tired when it got home? It had a 'hard drive'";
                break;
            case 3:
                myText = "Why do robots have summer holidays? To recharge their batteries";
                break;
            case 4:
                myText = "Why did the robot go to the doctor? Because it had a virus!";
                break;
            case 5:
                myText = "Why was the robot mad? People kept pushing its buttons";
                break;
            case 6:
                myText = "What do you call a pirate robot? Argh2-D2";
                break;
            case 7:
                myText = "Why did the robot cross the road? Because it was programmed by a chicken";
                break;
            case 8:
                myText = "Why did the robot go back to robot school? Because his skills were getting a little rusty!";
                break;
            case 9:
                myText = "What does a Robot's tomb say? Rust in Peace";
                break;
            case 10:
                myText = "What is a robots favorite kind of music? Heavy Metal";
                break;
            default:
                myText = "Why are robots shy? Because they have hardware and software but no underware";
        }
        
         // source: http://www.jokes4us.com/peoplejokes/robotjokes.html";
        
        response.tell(myText);
        return;
    },
    
    // BaxterJokeTwo Intent
    BaxterJokeTwo: function(intent, session, response) {
        var myText;
        console.log("in joke2");
        
        
        // Write out about Baxter
        myText = "Baxter said 'something cool'" ;

        //source: http://www.jokes4us.com/peoplejokes/robotjokes.html"
        response.tell(myText);
        return;
    },
    
    // BaxterJokeThree Intent
    BaxterJokeThree: function(intent, session, response) {
        var myText;
        console.log("in joke3");
        
        // Write out about Baxter
        myText = "Baxter said: 'good for you'";

        // source: http://www.jokes4us.com/peoplejokes/robotjokes.html";
        response.tell(myText);
        return;
    },
    
    // BaxterJokeFour Intent
    BaxterJokeFour: function(intent, session, response) {
        var myText;
        console.log("in joke4");
        
        // Write out about Baxter
        myText = "Baxter said: NOOOOOOOOO!!!";

        // source: http://www.jokes4us.com/peoplejokes/robotjokes.html";
        response.tell(myText);
        return;
    },
    
    // BaxterJokeFive Intent
    BaxterJokeFive: function(intent, session, response) {
        var myText;
        console.log("in joke5");
        
        // Write out about Baxter
        myText = "Baxter said: power";

        // source: http://www.jokes4us.com/peoplejokes/robotjokes.html";
        response.tell(myText);
        return;
    },
    
    // BaxterJokeSix Intent
    BaxterJokeSix: function(intent, session, response) {
        var myText;
        console.log("in joke6");
        
        // Write out about Baxter
        myText = "Baxter said: nothing, but force a smile";

        // source: http://www.jokes4us.com/peoplejokes/robotjokes.html";
        response.tell(myText);
        return;
    },
    
    // BaxterJokeSeven Intent
    BaxterJokeSeven: function(intent, session, response) {
        var myText;
        console.log("in joke7");
        
        // Write out about Baxter
        myText = "Baxter said: so you stop and look at me";

        // source: http://www.jokes4us.com/peoplejokes/robotjokes.html";
        response.tell(myText);
        return;
    },
    
    // BaxterRefusal
    BaxterRefusal: function(intent, session, response) {
        var myText;
        console.log("in refusal");
        
        // Write out about Baxter
        myText = "Baxter is thinking aout his existence, please give him commands that don't make him think";
        response.tell(myText);
        return;
    },

    // help intent
    help: function(intent, session, response) {
        response.ask("Please refer to the 'Baxter Alexa' instructions provided with this project, or for more help/info, please contact our CS Department Manager Nicole Preece at npreece@uidaho.edu");
    },
    default: function(intent, session, response) {
        response.ask("Please Try again, I didn't hear what you said");
    },

};


// Create the handler that responds to the Alexa Request.
// At the time you create a Lambda function you specify a handler, a function in your code, that AWS Lambda can invoke when the service executes your code
exports.handler = function(event, context) {




    // Create an instance of the Echobot skill.
    var baxteralexacontrol = new BaxterAlexaControl();

    baxteralexacontrol.execute(event, context);




};

