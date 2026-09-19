# WSL2 Üzerinde ArduPilot SITL, MAVProxy ve DroneKit Bağlantı Mimarisi

Bu belge, WSL2 (Windows Subsystem for Linux) ortamında ArduPilot SITL, Gazebo, MAVProxy ve DroneKit'in (Python) birbirleriyle nasıl iletişim kurduğunu ve geçmişte yaşanan bağlantı/timeout sorunlarının nasıl çözüldüğünü listeler. Lütfen ileride port değiştirmek isterseniz veya benzer bir "Operation already in progress" / "Timeout" hatası alırsanız bu yapıya dikkat edin.

## 1. Mimari ve Bağlantı Akışı

Simülasyonda sistemler şu sırayla birbirine bağlanır:

1. **ArduPilot SITL (ArduCopter):**
   - Başlatıldığında standart olarak **TCP 5760** portunu açar ve dinlemeye başlar. (MAVLink Master)
   
2. **MAVProxy (Yönlendirici / Köprü):**
   - `--master tcp:127.0.0.1:5760` ile **TCP 5760** portundan doğrudan SITL'e *client* (istemci) olarak bağlanır. 
   - SITL'den aldığı MAVLink telemetri verilerini `--out udp:127.0.0.1:14550` komutuyla **UDP 14550** portuna yayınlar.
   - *Kritik Detay:* WSL içerisinde arka planda (terminal olmadan) çalıştığı için mutlaka `--daemon` ve `--non-interactive` parametrelerini almalıdır, aksi halde anında çöker. Ayrıca SITL'in 5760 portunu açabilmesi için MAVProxy başlatılmadan önce **en az 5 saniye beklenmelidir**.
   
3. **DroneKit (drone_pose_controller.py):**
   - `connect('udp:127.0.0.1:14550', wait_ready=True)` komutu ile çalışır.
   - Kendi içinde **14550** UDP portunu dinlemeye (bind) başlar ve MAVProxy'nin gönderdiği veri paketlerini bekler.

---

## 2. Portların Özeti

| Bileşen | Kullandığı Port | Protokol | Yön | Görevi / Amacı |
| :--- | :--- | :--- | :--- | :--- |
| **SITL (ArduCopter)** | 5760 | TCP | Dinler (Listen) | Temel uçuş kontrolcüsü. Ana MAVLink bağlantısı. |
| **MAVProxy** | 5760 | TCP | Bağlanır (Client) | SITL'e bağlanarak veriyi çeker. |
| **MAVProxy** | 14550 | UDP | Gönderir (Out) | Çektiği veriyi DroneKit'e ulaştırır. |
| **DroneKit (Python)** | 14550 | UDP | Dinler (Bind) | Kontrolcü kodunuzun veriyi aldığı ve komut gönderdiği port. |

---

## 3. Karşılaşılan Hatalar ve Sebepleri

- **`[Errno 114] Operation already in progress` veya UDP Timeout Hataları:** 
  Bu hata, MAVProxy'nin çökmesinden dolayı DroneKit'in `14550` portundan veri alamaması (SITL'den kopması) durumunda ortaya çıktı. Çözüm, MAVProxy'i daemon modunda başlatmak ve başlatmadan önce SITL'in kendine gelmesi için 5 saniye (`sleep 5`) beklemek oldu.
  
- **Doğrudan TCP 5760'a DroneKit ile Bağlanamama:**
  WSL2 üzerinde `sim_vehicle.py` veya MAVProxy kullanmadan doğrudan SITL (5760) portuna DroneKit ile bağlanmak, `wait_ready` aşamasında parametrelerin alınamaması nedeniyle (timeout) takılmalara neden olur. Bu yüzden veri akışını stabilize eden **MAVProxy'nin arada (14550 UDP ile) kullanılması zorunludur.**

> **Not:** Eğer sistemi `./start_simulation.sh` üzerinden değil de manuel başlatacaksanız, sıralamanın yukarıdaki gibi (önce SITL, sonra MAVProxy, sonra Python) olduğundan emin olun.
