const express = require("express");
const http = require("http");
const https = require("https");
const fs = require("fs");
const path = require("path");
const mqtt = require("mqtt");
const { Server } = require("socket.io");

const HTTPS_PORT = Number(process.env.HTTPS_PORT || process.env.HTTP_PORT || 8090);
const REDIRECT_HTTP_PORT = Number(process.env.REDIRECT_HTTP_PORT || 8080);
const TLS_PFX_PATH = process.env.TLS_PFX_PATH || path.join(__dirname, "cert", "ruche-dashboard.pfx");
const TLS_PFX_PASSPHRASE = process.env.TLS_PFX_PASSPHRASE || "ruche-dashboard";
const MQTT_URL = process.env.MQTT_URL || "mqtt://broker.hivemq.com:1883";
const MQTT_TOPIC = process.env.MQTT_TOPIC || "ruches/telemetry";

const app = express();

let server;
let httpsEnabled = false;
if (fs.existsSync(TLS_PFX_PATH)) {
  const tlsOptions = {
    pfx: fs.readFileSync(TLS_PFX_PATH),
    passphrase: TLS_PFX_PASSPHRASE,
  };
  server = https.createServer(tlsOptions, app);
  httpsEnabled = true;
} else {
  server = http.createServer(app);
}

const io = new Server(server, {
  cors: { origin: "*" },
});

app.use(express.static("public"));
app.get("/api/status", (_req, res) => {
  res.json({
    mqtt_url: MQTT_URL,
    topic: MQTT_TOPIC,
    last: state.last,
    history_len: state.history.length,
  });
});

const state = {
  last: null,
  history: [],
};

function nowIso() {
  return new Date().toISOString();
}

function sanitizeTelemetry(payload) {
  if (!payload || typeof payload !== "object") return null;
  if (payload.heartbeat) return null;

  const weight = Number(payload.weight_g);
  const temp = Number(payload.temp_c);
  const hum = Number(payload.hum_pct);
  const batt = Number(payload.batt_pct);
  const rssi = Number(payload.rssi);
  const packet = Number(payload.packet);

  if (!Number.isFinite(weight)) return null;

  return {
    ts: nowIso(),
    packet: Number.isFinite(packet) ? packet : null,
    weight_g: weight,
    temp_c: Number.isFinite(temp) ? temp : null,
    hum_pct: Number.isFinite(hum) ? hum : null,
    batt_pct: Number.isFinite(batt) ? batt : null,
    rssi: Number.isFinite(rssi) ? rssi : null,
    raw: payload.raw ?? "",
  };
}

const mqttClient = mqtt.connect(MQTT_URL, {
  reconnectPeriod: 2000,
});

mqttClient.on("connect", () => {
  console.log(`[MQTT] Connecte: ${MQTT_URL}`);
  mqttClient.subscribe(MQTT_TOPIC, (err) => {
    if (err) {
      console.error("[MQTT] Erreur subscribe:", err.message);
    } else {
      console.log(`[MQTT] Abonne: ${MQTT_TOPIC}`);
    }
  });
});

mqttClient.on("error", (err) => {
  console.error("[MQTT] Erreur:", err.message);
});

mqttClient.on("message", (_topic, buffer) => {
  let payload;
  try {
    payload = JSON.parse(buffer.toString("utf8"));
  } catch (_e) {
    return;
  }
  const row = sanitizeTelemetry(payload);
  if (!row) return;

  state.last = row;
  state.history.push(row);
  if (state.history.length > 2000) {
    state.history.splice(0, state.history.length - 2000);
  }

  io.emit("telemetry", row);
});

io.on("connection", (socket) => {
  socket.emit("bootstrap", {
    last: state.last,
    history: state.history,
  });
});

server.listen(HTTPS_PORT, () => {
  if (httpsEnabled) {
    console.log(`[HTTPS] Dashboard: https://127.0.0.1:${HTTPS_PORT}`);
  } else {
    console.log(`[HTTP] Dashboard: http://127.0.0.1:${HTTPS_PORT}`);
    console.warn(`[TLS] PFX manquant: ${TLS_PFX_PATH}`);
  }
});

if (httpsEnabled) {
  const redirectApp = express();
  redirectApp.use((req, res) => {
    const host = (req.headers.host || "").split(":")[0] || "127.0.0.1";
    res.redirect(301, `https://${host}:${HTTPS_PORT}${req.originalUrl}`);
  });
  http.createServer(redirectApp).listen(REDIRECT_HTTP_PORT, () => {
    console.log(`[HTTP] Redirect: http://127.0.0.1:${REDIRECT_HTTP_PORT} -> https://127.0.0.1:${HTTPS_PORT}`);
  });
}
