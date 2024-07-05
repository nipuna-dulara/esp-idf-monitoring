const express = require('express');
const bodyParser = require('body-parser');
const mqtt = require('mqtt');

const app = express();
app.use(bodyParser.json());

const MQTT_SERVER = "mqtt://mqtt_server_uri"; // Replace with your MQTT server URI
const MQTT_TOPIC = "/sensor/warning";

// Connect to the MQTT broker
const mqttClient = mqtt.connect(MQTT_SERVER);

mqttClient.on('connect', () => {
    console.log(`Connected to MQTT broker at ${MQTT_SERVER}`);
    mqttClient.subscribe(MQTT_TOPIC, (err) => {
        if (err) {
            console.error(`Failed to subscribe to topic ${MQTT_TOPIC}`, err);
        } else {
            console.log(`Subscribed to topic ${MQTT_TOPIC}`);
        }
    });
});

mqttClient.on('message', (topic, message) => {
    if (topic === MQTT_TOPIC) {
        const payload = message.toString();
        console.log(`Received message: ${payload}`);
        // You can add more logic here to process the message
    }
});

mqttClient.on('error', (err) => {
    console.error('MQTT client error:', err);
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});
