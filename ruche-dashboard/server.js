const express = require("express");
const http = require("http");
const https = require("https");
const fs = require("fs");
const path = require("path");
const mqtt = require("mqtt");
const sqlite3 = require("sqlite3").verbose();
const { Server } = require("socket.io");

const HTTPS_PORT = Number(process.env.PORT || process.env.HTTPS_PORT || process.env.HTTP_PORT || 8090);
const REDIRECT_HTTP_PORT = Number(process.env.REDIRECT_HTTP_PORT || 8080);
const TLS_PFX_PATH = process.env.TLS_PFX_PATH || path.join(__dirname, "cert", "ruche-dashboard.pfx");
const TLS_PFX_PASSPHRASE = process.env.TLS_PFX_PASSPHRASE || "ruche-dashboard";
const MQTT_URL = process.env.MQTT_URL || "mqtt://broker.hivemq.com:1883";
const MQTT_TOPIC = process.env.MQTT_TOPIC || "ruches/telemetry";
const HISTORY_LIMIT = Number(process.env.HISTORY_LIMIT || 2000);
const DB_PATH = process.env.SQLITE_PATH || path.join(__dirname, "data", "history.sqlite3");
const DASH_USER = process.env.DASH_USER || "";
const DASH_PASS = process.env.DASH_PASS || "";
const authEnabled = DASH_USER.length > 0 && DASH_PASS.length > 0;

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
  allowRequest: (req, callback) => {
    if (!authEnabled) return callback(null, true);
    const ok = isAuthorized(req.headers.authorization);
    callback(null, ok);
  },
});

fs.mkdirSync(path.dirname(DB_PATH), { recursive: true });
const db = new sqlite3.Database(DB_PATH);
db.serialize(() => {
  db.run(`
    CREATE TABLE IF NOT EXISTS telemetry (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      ts TEXT NOT NULL,
      packet INTEGER,
      weight_g REAL NOT NULL,
      temp_c REAL,
      hum_pct REAL,
      batt_pct REAL,
      rssi INTEGER,
      raw TEXT
    )
  `);
});

app.get("/healthz", (_req, res) => {
  res.json({ ok: true });
});

if (authEnabled) {
  app.use((req, res, next) => {
    if (req.path === "/healthz") return next();
    if (isAuthorized(req.headers.authorization)) return next();
    res.set("WWW-Authenticate", 'Basic realm="Ruches Dashboard"');
    return res.status(401).send("Authentication required");
  });
}

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

function isAuthorized(authHeader) {
  if (!authEnabled) return true;
  if (!authHeader || !authHeader.startsWith("Basic ")) return false;
  try {
    const decoded = Buffer.from(authHeader.slice(6), "base64").toString("utf8");
    return decoded === `${DASH_USER}:${DASH_PASS}`;
  } catch (_e) {
    return false;
  }
}

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
  const rssi = Number(payload.rssi_dbm ?? payload.rssi);
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

db.all(
  `SELECT ts, packet, weight_g, temp_c, hum_pct, batt_pct, rssi, raw
   FROM telemetry
   ORDER BY id DESC
   LIMIT ?`,
  [HISTORY_LIMIT],
  (err, rows) => {
    if (err) {
      console.error("[DB] Erreur lecture historique:", err.message);
      return;
    }
    rows.reverse();
    state.history = rows.map((r) => ({
      ts: r.ts,
      packet: r.packet,
      weight_g: r.weight_g,
      temp_c: r.temp_c,
      hum_pct: r.hum_pct,
      batt_pct: r.batt_pct,
      rssi: r.rssi,
      raw: r.raw || "",
    }));
    state.last = state.history.length > 0 ? state.history[state.history.length - 1] : null;
    console.log(`[DB] Historique charge: ${state.history.length}`);
  }
);

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
    state.history.splice(0, state.history.length - HISTORY_LIMIT);
  }
  db.run(
    `INSERT INTO telemetry (ts, packet, weight_g, temp_c, hum_pct, batt_pct, rssi, raw)
     VALUES (?, ?, ?, ?, ?, ?, ?, ?)`,
    [row.ts, row.packet, row.weight_g, row.temp_c, row.hum_pct, row.batt_pct, row.rssi, row.raw]
  );
  db.run(
    `DELETE FROM telemetry
     WHERE id NOT IN (SELECT id FROM telemetry ORDER BY id DESC LIMIT ?)`,
    [HISTORY_LIMIT]
  );

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
