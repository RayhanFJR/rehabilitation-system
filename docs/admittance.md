# Penurunan Nilai Statis Kekakuan (K) dan Redaman (B) Virtual pada Kontrol Admittance Berbasis Model Hill-Zajac

## 1. Latar Belakang

Kontroler admittance orde pertama (M = 0) yang diimplementasikan pada manipulator paralel 3-RPS untuk rehabilitasi pergelangan kaki dirumuskan sebagai:

$$B \dot{Z} + K Z = F_{ext} \tag{1}$$

dengan $Z$ adalah perpindahan virtual (m), $F_{ext}$ gaya interaksi yang terukur pada load cell (N), serta $K$ (N/m) dan $B$ (N·s/m) masing-masing adalah kekakuan dan redaman virtual yang menentukan seberapa "lunak" robot merespons gaya dari pengguna.

Pada iterasi awal, nilai $K$ dan $B$ dihitung ulang di setiap siklus kendali (10 ms) sebagai fungsi dari gaya eksternal terukur, mengikuti prinsip aktivasi otot pada model Hill-Zajac. Pendekatan ini secara konseptual keliru: persamaan (1) mengasumsikan $K$ dan $B$ *time-invariant* (statis), sehingga menjadikannya fungsi dari $F_{ext}$ mengubah sistem menjadi *variable admittance control* — sebuah paradigma kendali yang berbeda dan memerlukan analisis kestabilan tersendiri yang tidak dicakup dalam desain ini. Dokumen ini menjabarkan penurunan nilai statis $K$ dan $B$ berdasarkan model Hill-Zajac (Zajac, 1989), dievaluasi pada satu titik operasi tetap, alih-alih dipilih secara empiris tanpa dasar perhitungan.

## 2. Model Hill-Zajac

Gaya yang dibangkitkan oleh elemen kontraktil otot (Zajac, 1989) dinyatakan sebagai:

$$F^M(t) = F_0 \left[ a(t)\, f_L(\tilde{l})\, f_v(\tilde{v}) \right] \tag{2}$$

dengan $F_0$ adalah gaya isometrik maksimal otot (N), $a(t) \in [0,1]$ aktivasi otot, $\tilde{l} = l/l_0$ panjang serat ternormalisasi terhadap panjang optimal $l_0$, dan $\tilde{v} = v/v_{max}$ kecepatan kontraksi ternormalisasi terhadap kecepatan maksimal $v_{max}$.

Kekakuan dan redaman lokal di sekitar suatu titik operasi diperoleh melalui linearisasi (turunan parsial) persamaan (2) terhadap panjang dan kecepatan (Sartori et al., dikutip dalam Millard et al., 2023):

$$K = \left.\frac{\partial F^M}{\partial l}\right|_{\tilde{l},\tilde{v}} = \frac{a F_0}{l_0}\, f_L'(\tilde{l})\, f_v(\tilde{v}) \tag{3}$$

$$B = \left.\frac{\partial F^M}{\partial v}\right|_{\tilde{l},\tilde{v}} = \frac{a F_0}{v_{max}}\, f_L(\tilde{l})\, f_v'(\tilde{v}) \tag{4}$$

### 2.1 Kurva Force-Length

Kurva force-length aktif didekati dengan fungsi Gaussian (Thelen, 2003; digunakan sebagai standar pada OpenSim):

$$f_L(\tilde{l}) = \exp\left(-\frac{(\tilde{l}-1)^2}{K_{act}}\right), \qquad K_{act} = 0.5 \tag{5}$$

$$f_L'(\tilde{l}) = -\frac{2(\tilde{l}-1)}{K_{act}}\, f_L(\tilde{l}) \tag{6}$$

**Catatan penting:** pada $\tilde{l} = 1$ (tepat di panjang optimal $l_0$), $f_L'(1) = 0$ karena titik tersebut merupakan puncak kurva. Evaluasi persamaan (3) tepat di $\tilde{l}=1$ akan selalu menghasilkan $K=0$ — ini adalah keterbatasan yang telah didokumentasikan pada pendekatan turunan langsung terhadap kurva force-length (lihat diskusi pada Millard et al., 2023, dan referensi Sartori et al. di dalamnya). Karena itu, titik evaluasi $\tilde{l}$ pada studi ini sengaja dipilih menyimpang dari $l_0$, merepresentasikan rentang gerak (ROM) latihan yang sesungguhnya, bukan posisi diam di panjang optimal.

### 2.2 Kurva Force-Velocity

Kurva force-velocity mengikuti persamaan hiperbola Hill dalam bentuk ternormalisasi:

$$f_v(\tilde{v}) = \frac{A_f(1+A_f)}{\tilde{v}+A_f} - A_f, \qquad A_f = 0.3 \tag{7}$$

Pada kondisi isometrik ($\tilde{v}=0$): $f_v(0) = 1$ (terverifikasi dari substitusi persamaan 7). Turunannya:

$$f_v'(\tilde{v}) = -\frac{A_f(1+A_f)}{(\tilde{v}+A_f)^2} \quad\Rightarrow\quad f_v'(0) = -\frac{1+A_f}{A_f} = -4.333 \tag{8}$$

## 3. Parameter Fisiologis

Parameter otot diambil dari basis data Delp et al. (1990), yang umum digunakan pada pemodelan muskuloskeletal ekstremitas bawah (mis. model OpenSim gait2392):

| Otot | $F_0$ (N) | $l_0$ (m) |
|---|---|---|
| Tibialis Anterior (dorsofleksor) | 603 | 0.098 |
| Gastrocnemius medial | 1113 | 0.045 |
| Gastrocnemius lateral | 488 | 0.064 |
| Soleus | 2839 | 0.030 |

Untuk kelompok plantarfleksor (triceps surae = gastrocnemius medial + lateral + soleus), $F_0$ dan $l_0$ digabung:

$$F_{0,triceps} = \sum F_0 = 4440 \text{ N}$$

$$l_{0,triceps} = \frac{\sum F_{0,i}\, l_{0,i}}{\sum F_{0,i}} = 0.0375 \text{ m (rata-rata terbobot gaya)}$$

Kecepatan kontraksi maksimal diasumsikan mengikuti pendekatan standar Zajac/Winters:

$$v_{max} = 10\, l_0 \; \text{/s} \tag{9}$$

## 4. Titik Operasi Berdasarkan ROM Latihan Aktual

Nilai statis $K$ dan $B$ memerlukan satu titik operasi tetap sebagai dasar linearisasi. Alih-alih diasumsikan begitu saja, titik operasi berikut diturunkan dari rentang gerak (ROM) aktual trajectory latihan robot:

| Arah gerak | ROM (°) | Titik tengah (°) |
|---|---|---|
| Dorsofleksi | 20.3 – 29.8 | 25.05 |
| Plantarfleksi | 37.6 – 45.8 | 41.70 |

Total sweep sudut yang dilalui otot selama satu siklus latihan penuh (dari batas plantarfleksi ke batas dorsofleksi):

$$\Delta\theta_{total} = 25.05° + 41.70° = 66.75° = 1.1652 \text{ rad}$$

Dengan asumsi $l_0$ berada di tengah rentang sweep tersebut, deviasi sudut representatif terhadap $l_0$ adalah setengah dari total sweep:

$$\Delta\theta = \frac{\Delta\theta_{total}}{2} = 33.375° = 0.5826 \text{ rad}$$

### 4.1 Konversi Sudut Sendi ke Deviasi Panjang Serat

Perubahan panjang unit muskulotendon akibat rotasi sendi didekati dengan:

$$\Delta l_{MTU} = r \cdot \Delta\theta \tag{11}$$

dengan $r$ adalah *moment arm* otot terhadap sumbu sendi ankle (Maganaris et al., 1999):

| Otot | $r$ (m) |
|---|---|
| Tibialis Anterior | 0.035 |
| Triceps Surae (Achilles tendon) | 0.050 |

**Koreksi gearing tendon-fascicle.** Persamaan (11) mengasumsikan seluruh perubahan panjang MTU diteruskan ke serat otot (fascicle) — asumsi ini cukup wajar untuk Tibialis Anterior yang tendonnya relatif pendek dan kaku ($g_{TA} \approx 1.0$). Namun untuk triceps surae, Achilles tendon yang panjang dan lentur menyerap sebagian besar perubahan panjang MTU: pengukuran in-vivo (Hoang et al.; studi *in-vivo* soleus saat berjalan/berlari) menunjukkan **hanya sekitar 27–35% dari perubahan panjang MTU yang benar-benar tercermin pada perubahan panjang serat**, sisanya diserap regangan tendon. Tanpa koreksi ini, deviasi panjang serat triceps surae akan jatuh jauh di luar jangkauan valid kurva force-length Gaussian (menghasilkan $\tilde{l}\approx0.22$, secara fisiologis tidak masuk akal untuk rentang ROM yang diberikan).

$$\Delta l_{fascicle} = g \cdot r \cdot \Delta\theta \tag{12}$$

| Otot | $g$ | Sumber |
|---|---|---|
| Tibialis Anterior | 1.0 | asumsi (tendon relatif kaku) |
| Triceps Surae | 0.30 | Hoang et al.; gearing MTU-fascicle in-vivo ~27–35% |

### 4.2 Hasil Perhitungan $\tilde{l}$

$$\tilde{l} = 1 - \frac{\Delta l_{fascicle}}{l_0}$$

**Tibialis Anterior:**
$$\Delta l_{TA} = 1.0 \times 0.035 \times 0.5826 = 0.02039 \text{ m} \quad\Rightarrow\quad \tilde{l}_{TA} = 1 - \frac{0.02039}{0.098} = 0.792$$

**Triceps Surae:**
$$\Delta l_{tri} = 0.30 \times 0.050 \times 0.5826 = 0.00874 \text{ m} \quad\Rightarrow\quad \tilde{l}_{tri} = 1 - \frac{0.00874}{0.0375} = 0.767$$

Titik operasi lengkap: $a = 0.3$ (aktivasi nominal), $\tilde{v}=0$ (isometrik, konsisten dengan asumsi quasi-statis integrator Backward Euler).

## 5. Perhitungan K dan B

**Tibialis Anterior** ($\tilde{l}=0.792$): $f_L(0.792) = 0.9170$, $f_L'(0.792) = 0.7633$

$$K_{TA} = \frac{0.3 \times 603}{0.098} \times 0.7633 \times 1 = 1410 \text{ N/m}$$
$$B_{TA} = \frac{0.3 \times 603}{0.98} \times 0.9170 \times 4.333 = 734 \text{ N·s/m}$$

**Triceps Surae** ($\tilde{l}=0.767$): $f_L(0.767) = 0.8971$, $f_L'(0.767) = 0.8362$

$$K_{tri} = \frac{0.3 \times 4440}{0.0375} \times 0.8362 \times 1 = 29703 \text{ N/m}$$
$$B_{tri} = \frac{0.3 \times 4440}{0.375} \times 0.8971 \times 4.333 = 13810 \text{ N·s/m}$$

Nilai statis akhir diambil sebagai rata-rata dorsofleksor–plantarfleksor:

$$K_{adm} = \frac{1410+29703}{2} \approx 15556 \text{ N/m}, \qquad B_{adm} = \frac{734+13810}{2} \approx 7272 \text{ N·s/m}$$

$$\tau = \frac{B_{adm}}{K_{adm}} \approx 0.47 \text{ s}$$

## 6. Interpretasi Konstanta Waktu

Persamaan (1) dapat ditulis ulang sebagai sistem orde pertama standar:

$$\dot{Z} + \frac{1}{\tau} Z = \frac{F_{ext}}{B}, \qquad \tau = \frac{B}{K} \tag{10}$$

Untuk masukan gaya konstan (step), respons perpindahan virtual adalah $Z(t) = Z_{ss}(1-e^{-t/\tau})$ dengan $Z_{ss}=F_{ext}/K$. Nilai $\tau \approx 0.47$ s berarti waktu settling (≈98%) sekitar $4\tau \approx 1.9$ s — cukup responsif untuk kontrol admittance real-time pada perangkat keras ini.

Perlu dicatat, parameter aktivasi $a$ muncul pada pembilang $K$ maupun $B$, sehingga tereliminasi pada rasio $\tau=B/K$. Nilai $\tau$ murni ditentukan oleh $l_0/v_{max}$ (tetap $0.1$ s untuk asumsi persamaan 9) dan rasio $f_L(\tilde{l})/f_L'(\tilde{l})$, yang bergantung pada seberapa jauh titik evaluasi $\tilde{l}$ dari $l_0$.

## 7. Keterbatasan dan Asumsi yang Perlu Dijustifikasi

1. Titik operasi $\tilde{l}$ kini diturunkan dari ROM aktual trajectory latihan (§4), namun asumsi $l_0$ berada tepat di **tengah** rentang sweep masih perlu divalidasi — bila posisi netral/optimal otot secara fisiologis tidak simetris terhadap ROM latihan, deviasi $\Delta\theta$ perlu dihitung ulang dari posisi $l_0$ yang sebenarnya.
2. Aktivasi nominal $a=0.3$ masih merupakan asumsi desain (representasi kontraksi sedang), bukan hasil pengukuran EMG langsung pada subjek.
3. Faktor gearing tendon $g=0.30$ untuk triceps surae diambil dari studi in-vivo pada populasi umum (Hoang et al.; rentang literatur 27–35%); nilai ini dapat bervariasi antar individu tergantung kekakuan tendon Achilles masing-masing subjek/pasien.
4. Rata-rata dorsofleksor–plantarfleksor adalah penyederhanaan; jika robot didesain untuk resistansi dominan pada satu arah gerak (mis. dorsofleksi untuk kasus foot-drop), nilai $K_{TA}$, $B_{TA}$ saja lebih representatif.
5. Validitas dimensi $K$ (N/m) dan $B$ (N·s/m) bergantung pada kalibrasi load cell ke satuan Newton dan konsistensi satuan meter pada $Z_{adm}$ (dikonversi ke mm hanya pada antarmuka ke loop kendali posisi).

## Referensi

Delp, S. L., Loan, J. P., Hoy, M. G., Zajac, F. E., Topp, E. L., & Rosen, J. M. (1990). An interactive graphics-based model of the lower extremity to study orthopaedic surgical procedures. *IEEE Transactions on Biomedical Engineering*, 37(8), 757–767.

Hoang, P. D., Herbert, R. D., Todd, G., Gorman, R. B., & Gandevia, S. C. (2007). Passive mechanical properties of human gastrocnemius muscle-tendon units, muscle fascicles and tendons in vivo. *Journal of Experimental Biology*, 210(23), 4159–4168.

Maganaris, C. N., Baltzopoulos, V., & Sargeant, A. J. (1999). Changes in Achilles tendon moment arm from rest to maximum isometric plantarflexion: in vivo observations in man. *Journal of Physiology*, 510(3), 977–985.

Thelen, D. G. (2003). Adjustment of muscle mechanics model parameters to simulate dynamic contractions in older adults. *Journal of Biomechanical Engineering*, 125(1), 70–77.

Zajac, F. E. (1989). Muscle and tendon: properties, models, scaling, and application to biomechanics and motor control. *Critical Reviews in Biomedical Engineering*, 17(4), 359–411.
