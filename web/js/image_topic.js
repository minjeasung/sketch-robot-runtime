(function(root) {
  "use strict";
  function createImageTopic(options, library = root.ROSLIB) {
    const {reliable, ...topicOptions} = options;
    const topic = new library.Topic({messageType: "sensor_msgs/Image", throttle_rate: 200,
      queue_length: 1, ...topicOptions});
    if (reliable || options.name === "/perception/wall_front_view") {
      // The projector is a reliable publisher. The bundled ROSLIB predates
      // the rosbridge QoS option; add it before its reconnect wrapper captures
      // the packet, without changing subscriptions to native camera drivers.
      const send = topic.callForSubscribeAndAdvertise;
      topic.callForSubscribeAndAdvertise = function(message) {
        if (message.op === "subscribe") message.qos = {
          reliability: "reliable", durability: "volatile", history: "keep_last", depth: 1,
        };
        send.call(this, message);
      };
    }
    return topic;
  }
  root.createImageTopic = createImageTopic;
  if (typeof module === "object" && module.exports) module.exports = {createImageTopic};
})(typeof globalThis !== "undefined" ? globalThis : this);
