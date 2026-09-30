"use strict";

const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

// Only browser drawing and ROS transport are replaced. Production scripts own
// every handler, state transition, outgoing message, and disabled control.
function loadUI(page = "index.html", fetch) {
  const html = fs.readFileSync(path.join(__dirname, "..", page), "utf8");
  const elements = new Map();
  const radios = [];
  class Element {
    constructor(tag = "div") {
      this.tagName = tag;
      this.children = [];
      this.listeners = {};
      this.style = {};
      this.dataset = {};
      this.value = "";
      this.textContent = "";
      this.disabled = false;
      this.hidden = false;
      this.width = 1280;
      this.height = 720;
      this.classList = { add() {}, remove() {}, toggle() {} };
    }
    set checked(value) {
      if (value && this.type === "radio") radios.filter(r => r.name === this.name).forEach(r => { r._checked = false; });
      this._checked = value;
    }
    get checked() { return this._checked === true; }
    get options() { return this.children; }
    addEventListener(name, fn) { (this.listeners[name] ||= []).push(fn); }
    async fire(name, extra = {}) {
      for (const fn of this.listeners[name] || []) await fn({ target: this, preventDefault() {}, ...extra });
    }
    append(...children) { this.children.push(...children); }
    add(child) { this.append(child); }
    replaceChildren(...children) { this.children = children; }
    setAttribute(name, value) { this[name] = value; }
    getContext() { return new Proxy({}, { get: (obj, key) => obj[key] || (() => {}), set: (obj, key, value) => { obj[key] = value; return true; } }); }
    getBoundingClientRect() { return { left: 0, top: 0, width: this.width, height: this.height }; }
    setPointerCapture() {}
    releasePointerCapture() {}
  }
  const attribute = (tag, name) => tag.match(new RegExp(`\\b${name}="([^"]*)"`))?.[1];
  for (const [tag] of html.matchAll(/<[a-z][^>]*>/gi)) {
    const id = attribute(tag, "id");
    const name = attribute(tag, "name");
    if (!id && !["workflow-mode", "sketch-mode"].includes(name)) continue;
    const element = new Element(tag.match(/^<(\w+)/)[1]);
    Object.assign(element, { id, name, type: attribute(tag, "type"), value: attribute(tag, "value") || "" });
    if (element.type === "radio") radios.push(element);
    element.checked = /\bchecked\b/.test(tag);
    element.disabled = /\bdisabled\b/.test(tag);
    if (id) elements.set(id, element);
  }
  for (const [whole, id] of html.matchAll(/<select[^>]*id="([^"]+)"[^>]*>[\s\S]*?<\/select>/g)) {
    elements.get(id).value = whole.match(/<option[^>]*value="([^"]*)"/)?.[1] || "";
  }
  function queryAll(selector) {
    if (selector === ".process-row button") return [];
    return radios.filter(radio => selector.split(",").some(part => {
      const name = part.match(/name="([^"]+)"/)?.[1];
      const value = part.match(/value="([^"]+)"/)?.[1];
      return radio.name === name && (!value || radio.value === value) && (!part.includes(":checked") || radio.checked);
    }));
  }
  const published = [];
  const topics = [];
  let ros;
  class Ros {
    constructor() { ros = this; this.listeners = {}; }
    on(event, fn) { (this.listeners[event] ||= []).push(fn); }
    emit(event) { for (const fn of this.listeners[event] || []) fn(); }
  }
  class Topic {
    constructor(options) { Object.assign(this, options); this.callbacks = []; topics.push(this); }
    subscribe(fn) { this.callbacks.push(fn); }
    unsubscribe() { this.callbacks = []; }
    publish(message) { published.push({ name: this.name, message: JSON.parse(JSON.stringify(message)) }); }
  }
  const context = vm.createContext({
    document: {
      getElementById: id => {
        if (!elements.has(id)) throw new Error(`Missing UI element ${id}`);
        return elements.get(id);
      },
      querySelectorAll: queryAll, querySelector: selector => queryAll(selector)[0],
      createElement: tag => new Element(tag), createTextNode: text => ({ textContent: text }), addEventListener() {},
    },
    ROSLIB: { Ros, Topic, Message: class { constructor(value) { Object.assign(this, value); } } },
    location: { hostname: "localhost", href: "http://localhost:8000/" },
    confirm: () => true, setInterval() {}, setTimeout() { return 1; }, clearTimeout() {},
    atob: value => Buffer.from(value, "base64").toString("binary"),
    ImageData: class { constructor(data, width, height) { Object.assign(this, { data, width, height }); } },
    Option: class extends Element { constructor(text, value) { super("option"); this.textContent = text; this.value = value; } },
    URL, fetch,
  });
  context.window = context;
  for (const [, src] of html.matchAll(/<script src="([^"]+)"/g)) {
    if (src.endsWith("roslib.min.js")) continue;
    const name = src.replace(/^\/sketch\//, "").split("?")[0];
    vm.runInContext(fs.readFileSync(path.join(__dirname, "..", name), "utf8"), context, { filename: name });
  }
  return {
    element: id => elements.get(id), queryAll, published, ros,
    evaluate: code => vm.runInContext(code, context),
    emit(name, payload, raw = false) {
      const message = raw ? payload : { data: JSON.stringify(payload) };
      for (const topic of topics.filter(t => t.name === name)) for (const fn of topic.callbacks) fn(message);
    },
    frame(name) {
      this.emit(name, { width: 100, height: 80, encoding: "rgb8", data: Buffer.alloc(24000).toString("base64") }, true);
    },
    async rectangle() {
      const canvas = elements.get("sketch-canvas");
      await canvas.fire("pointerdown", { pointerId: 1, clientX: 10, clientY: 10 });
      await canvas.fire("pointermove", { pointerId: 1, clientX: 90, clientY: 70 });
      await canvas.fire("pointerup", { pointerId: 1, clientX: 90, clientY: 70 });
    },
  };
}

module.exports = { loadUI };
