use anyhow::Result;
use log::*;
use esp_idf_svc::{
    mqtt::client::{
    EspMqttClient, EventPayload, MqttClientConfiguration, QoS},
    tls::X509,
};
use std::{sync::{atomic::{AtomicBool, Ordering}, Arc}, thread};
use std::ffi::CStr;
use std::time::Duration;
use std::collections::VecDeque;
pub struct Mqtt {
    client: EspMqttClient<'static>,
    connected: Arc<AtomicBool>,
}

const CA_CERT: &CStr = unsafe{
    CStr::from_bytes_with_nul_unchecked(concat!(include_str!("../fullchain.pem"), "\0").as_bytes())
};
impl Mqtt {
    /// Create a new TLS-secured MQTT client
    pub fn new_mqtt(broker_url: &str, client_id: &str, user: &str, pass: &str) -> Result<Self> {


        let mqtt_config = MqttClientConfiguration {
            client_id: Some(client_id),
            username: Some(user),
            password: Some(pass),
            server_certificate: Some(X509::pem(CA_CERT)),
            keep_alive_interval: Some(Duration::from_secs(60)),
            ..Default::default()
        };

        info!("Attempting to create MQTT client...");
        info!("Broker URL: {}", broker_url);

        let connected = Arc::new(AtomicBool::new(false));
        let connected_clone = connected.clone();

        let (mut client, mut connection) = EspMqttClient::new(
            broker_url,
            &mqtt_config,
        )?;

        info!("MQTT client created successfully!");

        thread::spawn(move || {
            let mut message_queue: VecDeque<(String, Vec<u8>)> = VecDeque::new();

            while let Ok(event) = connection.next() {
                match event.payload() {
                    EventPayload::Connected(_) => {
                        info!("MQTT Connected");
                        connected_clone.store(true, Ordering::SeqCst);

                        // publish inside thread if needed

                        /* // Flush queued messages
                        while let Some((topic, payload)) = message_queue.pop_front() {
                            match client.publish(&topic, QoS::AtLeastOnce, false, &payload) {
                                Ok(_) => info!("Queued message published successfully"),
                                Err(e) => {
                                    warn!("Failed to publish queued message: {:?}, putting back in queue", e);
                                    message_queue.push_front((topic, payload));
                                    break; // stop flushing for now
                                }
                            }

                        } */
                    }
                    EventPayload::Disconnected => {
                        warn!("MQTT Disconnected, will queue messages temporarily...");
                        warn!("Retrying momentarilly...");
                        connected_clone.store(false, Ordering::SeqCst);
                        // trigger reconnect
                    }
                    EventPayload::Published(id) => info!("MQTT Publish Message {} confirmed", id),
                    EventPayload::Error(e) => error!("MQTT error: {:?}", e),
                    _ => {}
                }
            }
        });

        Ok(Self {client, connected})
    }

    // Expose the flag safely
    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    pub fn publish(&mut self, topic: &str, payload: &[u8]) -> Result<()> {
        info!("Attempting to publish message to topic...");
        self.client.publish(topic, QoS::AtLeastOnce, false, payload)?;
        info!("Initial message published successfully!");
        Ok(())
    }

    pub fn subscribe(&mut self, topic: &str) -> Result<()> {
        self.client.subscribe(topic, QoS::AtMostOnce)?;
        Ok(())
    }
}
