//! The ESP itself, as the command service asks for it (`esp_tool_core::svc::Platform`): the Wi-Fi
//! station's state for `ESP_INFO`.

use embassy_net::Stack;
use espsvc::WifiState;

use esp_tool_core::svc::Platform;

/// The firmware's platform.
pub struct EspPlatform {
    stack: Stack<'static>,
}

impl EspPlatform {
    /// The platform over the station's IP stack.
    pub fn new(stack: Stack<'static>) -> Self {
        EspPlatform { stack }
    }
}

impl Platform for EspPlatform {
    fn wifi(&self) -> (WifiState, [u8; 4]) {
        if !self.stack.is_link_up() {
            return (WifiState::Down, [0; 4]);
        }
        match self.stack.config_v4() {
            Some(c) => (WifiState::Up, c.address.address().octets()),
            None => (WifiState::Joined, [0; 4]),
        }
    }
}
