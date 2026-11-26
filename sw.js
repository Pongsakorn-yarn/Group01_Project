/**
 * Service Worker สำหรับ 4-DOF Robotic Arm Digital Twin (PWA)
 * ใช้กลยุทธ์ Network-First เพื่อให้ได้อัปเดตล่าสุดเสมอ และ fallback ไปที่แคชเมื่อออฟไลน์
 */
const CACHE_NAME = 'arm-4dof-twin-v15';
const CORE_ASSETS = [
  './',
  './index.html',
  './dashboard.html',
  './diagnostics.html',
  './css/style.css',
  './css/pages.css',
  './js/factory_bus.js',
  './js/app.js',
  './js/dashboard.js',
  './js/diagnostics.js',
  './js/arm_cad_data.js',
  './manifest.json',
  './assets/icon.svg'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(CORE_ASSETS)).then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((k) => k !== CACHE_NAME).map((k) => caches.delete(k)))
    ).then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (event) => {
  if (event.request.method !== 'GET') return;
  const url = new URL(event.request.url);
  if (url.origin !== self.location.origin) return;

  event.respondWith(
    fetch(event.request)
      .then((response) => {
        if (response && response.status === 200 && response.type === 'basic') {
          const copy = response.clone();
          caches.open(CACHE_NAME).then((cache) => cache.put(event.request, copy));
        }
        return response;
      })
      .catch(() => caches.match(event.request))
  );
});
