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

$$K = \frac{\partial F^M}{\partial l}\Bigg|_{\tilde{l},\,\tilde{v}} = \frac{a F_0}{l_0}\, f_L'(\tilde{l})\, f_v(\tilde{v}) \tag{3}$$

$$B = \frac{\partial F^M}{\partial v}\Bigg|_{\tilde{l},\,\tilde{v}} = \frac{a F_0}{v_{max}}\, f_L(\tilde{l})\, f_v'(\tilde{v}) \tag{4}$$

### 2.1 Kurva Force-Length

Kurva force-length aktif didekati dengan fungsi Gaussian (Thelen, 2003; digunakan sebagai standar pada OpenSim):

$$f_L(\tilde{l}) = \exp\left(-\frac{(\tilde{l}-1)^2}{K_{act}}\right) \tag{5}$$

dengan $K_{act} = 0.5$.

$$f_L'(\tilde{l}) = -\frac{2(\tilde{l}-1)}{K_{act}}\, f_L(\tilde{l}) \tag{6}$$

**Catatan penting:** pada $\tilde{l} = 1$ (tepat di panjang optimal $l_0$), $f_L'(1) = 0$ karena titik tersebut merupakan puncak kurva. Evaluasi persamaan (3) tepat di $\tilde{l}=1$ akan selalu menghasilkan $K=0$ — ini adalah keterbatasan yang telah didokumentasikan pada pendekatan turunan langsung terhadap kurva force-length (lihat diskusi pada Millard et al., 2023, dan referensi Sartori et al. di dalamnya). Karena itu, titik evaluasi $\tilde{l}$ pada studi ini sengaja dipilih menyimpang dari $l_0$, merepresentasikan rentang gerak (ROM) latihan yang sesungguhnya, bukan posisi diam di panjang optimal.

### 2.2 Kurva Force-Velocity

Kurva force-velocity mengikuti persamaan hiperbola Hill dalam bentuk ternormalisasi:

$$f_v(\tilde{v}) = \frac{A_f(1+A_f)}{\tilde{v}+A_f} - A_f \tag{7}$$

dengan $A_f = 0.3$.

Pada kondisi isometrik ($\tilde{v}=0$): $f_v(0) = 1$ (terverifikasi dari substitusi persamaan 7). Turunannya:

$$f_v'(\tilde{v}) = -\frac{A_f(1+A_f)}{(\tilde{v}+A_f)^2} \tag{8}$$

Pada $\tilde{v}=0$: $f_v'(0) = -(1+A_f)/A_f = -4.333$.

## 3. Parameter Fisiologis

Parameter otot diambil dari basis data Delp et al. (1990), yang umum digunakan pada pemodelan muskuloskeletal ekstremitas bawah (mis. model OpenSim gait2392):

| Otot | $F_0$ (N) | $l_0$ (m) |
|---|---|---|
| Tibialis Anterior (dorsofleksor) | 603 | 0.098 |
| Gastrocnemius medial | 1113 | 0.045 |
| Gastrocnemius lateral | 488 | 0.064 |
| Soleus | 2839 | 0.030 |

Untuk kelompok plantarfleksor (triceps surae = gastrocnemius medial + lateral + soleus), $F_0$ dan $l_0$ digabung:

$$F_{0,triceps} = \sum F_0 = 4440 \text{ N} \tag{9}$$

$$l_{0,triceps} = \frac{\sum F_{0,i}\, l_{0,i}}{\sum F_{0,i}} = 0.0375 \text{ m} \tag{10}$$

(rata-rata terbobot gaya)

Kecepatan kontraksi maksimal diasumsikan mengikuti pendekatan standar Zajac/Winters:

$$v_{max} = 10\, l_0 \text{ /s} \tag{11}$$

## 4. Titik Operasi

Nilai statis $K$ dan $B$ memerlukan satu titik operasi tetap sebagai dasar linearisasi. Titik operasi berikut dipilih sebagai asumsi desain awal dan **perlu divalidasi ulang** terhadap protokol latihan aktual:

- Aktivasi nominal: $a = 0.3$ (representasi kontraksi sedang selama sesi latihan)
- Deviasi panjang: $\tilde{l} = 0.80$ (20% dari $l_0$, merepresentasikan eksursi ROM latihan; dipilih menjauh dari $\tilde{l}=1$ untuk menghindari $K=0$, lihat §2.1)
- Kecepatan: $\tilde{v} = 0$ (isometrik, konsisten dengan asumsi quasi-statis pada integrator Backward Euler yang digunakan)

## 5. Perhitungan

Dengan $\tilde{l}=0.80$, diperoleh $f_L(0.80) = \exp(-0.08) = 0.9231$ dan $f_L'(0.80) = 0.7385$.

**Tibialis Anterior:**

$$K_{TA} = \frac{0.3 \times 603}{0.098} \times 0.7385 \times 1 = 1364.0 \text{ N/m}$$

$$B_{TA} = \frac{0.3 \times 603}{0.98} \times 0.9231 \times 4.333 = 738.4 \text{ N·s/m}$$

**Triceps Surae:**

$$K_{tri} = \frac{0.3 \times 4440}{0.0375} \times 0.7385 \times 1 = 26236.9 \text{ N/m}$$

$$B_{tri} = \frac{0.3 \times 4440}{0.375} \times 0.9231 \times 4.333 = 14208.4 \text{ N·s/m}$$

Nilai statis akhir diambil sebagai rata-rata dorsofleksor–plantarfleksor:

$$K_{adm} = \frac{K_{TA}+K_{tri}}{2} \approx 13800 \text{ N/m}$$

$$B_{adm} = \frac{B_{TA}+B_{tri}}{2} \approx 7473 \text{ N·s/m}$$

$$\tau = \frac{B_{adm}}{K_{adm}} \approx 0.54 \text{ s}$$

## 6. Interpretasi Konstanta Waktu

Persamaan (1) dapat ditulis ulang sebagai sistem orde pertama standar:

$$\dot{Z} + \frac{1}{\tau} Z = \frac{F_{ext}}{B} \tag{12}$$

dengan $\tau = B/K$.

Untuk masukan gaya konstan (step), respons perpindahan virtual adalah $Z(t) = Z_{ss}(1-e^{-t/\tau})$ dengan $Z_{ss}=F_{ext}/K$. Nilai $\tau \approx 0.54$ s berarti waktu settling (≈98%) sekitar $4\tau \approx 2.2$ s — cukup responsif untuk kontrol admittance real-time pada perangkat keras ini.

Perlu dicatat, parameter aktivasi $a$ muncul pada pembilang $K$ maupun $B$, sehingga tereliminasi pada rasio $\tau=B/K$. Nilai $\tau$ murni ditentukan oleh $l_0/v_{max}$ (tetap $0.1$ s untuk asumsi persamaan 11) dan rasio $f_L(\tilde{l})/f_L'(\tilde{l})$, yang bergantung pada seberapa jauh titik evaluasi $\tilde{l}$ dari $l_0$.

## 7. Keterbatasan dan Asumsi yang Perlu Dijustifikasi

1. Titik operasi $a=0.3$ dan $\tilde{l}=0.80$ merupakan asumsi desain, bukan hasil pengukuran langsung pada subjek/latihan. Jika tersedia data ROM aktual (derajat dorsofleksi/plantarfleksi per sesi latihan), nilai $\tilde{l}$ sebaiknya dikonversi dari data tersebut.
2. Rata-rata dorsofleksor–plantarfleksor adalah penyederhanaan; jika robot didesain untuk resistansi dominan pada satu arah gerak (mis. dorsofleksi untuk kasus foot-drop), nilai $K_{TA}$, $B_{TA}$ saja lebih representatif.
3. Validitas dimensi $K$ (N/m) dan $B$ (N·s/m) bergantung pada kalibrasi load cell ke satuan Newton dan konsistensi satuan meter pada $Z_{adm}$ (dikonversi ke mm hanya pada antarmuka ke loop kendali posisi).

## Referensi

Delp, S. L., Loan, J. P., Hoy, M. G., Zajac, F. E., Topp, E. L., & Rosen, J. M. (1990). An interactive graphics-based model of the lower extremity to study orthopaedic surgical procedures. *IEEE Transactions on Biomedical Engineering*, 37(8), 757–767.

Thelen, D. G. (2003). Adjustment of muscle mechanics model parameters to simulate dynamic contractions in older adults. *Journal of Biomechanical Engineering*, 125(1), 70–77.

Zajac, F. E. (1989). Muscle and tendon: properties, models, scaling, and application to biomechanics and motor control. *Critical Reviews in Biomedical Engineering*, 17(4), 359–411.