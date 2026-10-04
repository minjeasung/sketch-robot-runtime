(function(root) {
  "use strict";
  function createImageTopic(options, library = root.ROSLIB) {
    const {reliable, ...topicOptions} = options;
    const topic = new library.Topic({messageType: "sensor_msgs/Image", throttle_rate: 200,
      queue_length: 1, ...topicOptions});
    if (topicOptions.reconnect_on_close === false) {
      // This bundled ROSLIB leaves the no-replay sender unbound. Bind it to
      // Ros before wrapping QoS; the app owns preview connection recovery.
      topic.callForSubscribeAndAdvertise = options.ros.callOnConnection.bind(options.ros);
    }
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
  function decodePreview(msg) {
    return new Promise((resolve, reject) => {
      const mime = /png/i.test(msg.format) ? "image/png" : "image/jpeg";
      const image = new root.Image();
      image.onload = () => resolve(image);
      image.onerror = () => reject(new Error("미리보기 영상 디코딩 실패"));
      image.src = `data:${mime};base64,${msg.data}`;
    });
  }

  function createPreviewStream({ros, name, reliable, onRaw, onPreview, onError}, library = root.ROSLIB) {
    let active = true, usable = false, raw = null, pending = null, decoding = false;
    const compressed = createImageTopic({ros, name: `${name}/compressed`, reliable: true,
      messageType: "sensor_msgs/CompressedImage", reconnect_on_close: false}, library);
    const fallback = root.setTimeout(() => {
      if (!active || usable) return;
      // Older servers/native camera drivers may not publish compressed images.
      raw = createImageTopic({ros, name, reliable, reconnect_on_close: false}, library);
      raw.subscribe(msg => { if (active && !usable) onRaw(msg); });
    }, 1500);

    async function drain() {
      decoding = true;
      while (active && pending) {
        const msg = pending; pending = null;
        try {
          const image = await decodePreview(msg);
          if (!active) break;
          onPreview(image);
          usable = true;
          root.clearTimeout(fallback);
          if (raw) { raw.unsubscribe(); raw = null; }
        } catch (error) { if (active && onError) onError(error); }
      }
      decoding = false;
    }
    compressed.subscribe(msg => {
      if (!active) return;
      pending = msg; // At most one decode and one latest pending frame.
      if (!decoding) void drain();
    });
    return {unsubscribe() {
      active = false; pending = null;
      root.clearTimeout(fallback);
      compressed.unsubscribe();
      if (raw) { raw.unsubscribe(); raw = null; }
    }};
  }
  root.createImageTopic = createImageTopic;
  root.createPreviewStream = createPreviewStream;
  if (typeof module === "object" && module.exports) module.exports = {createImageTopic, createPreviewStream};
})(typeof globalThis !== "undefined" ? globalThis : this);
