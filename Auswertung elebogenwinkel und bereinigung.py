import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_and_prepare_data(csv_file, person_name):
    data = pd.read_csv(csv_file, header=None, names=['RawData'])
    data[['Time', 'Elbogenwinkel Type', 'Value']] = data['RawData'].str.extract(
        r'(\d+)\s+elbogenwinkel\s+(links|rechts):\s*([\d\.]+)'
    )
    data['Time'] = pd.to_numeric(data['Time'], errors='coerce')
    data['Value'] = pd.to_numeric(data['Value'], errors='coerce')
    data['Person'] = person_name
    data = data.dropna(subset=['Time', 'Value', 'Elbogenwinkel Type'])
    return data

def remove_outliers_iqr(df):
    Q1 = df['Value'].quantile(0.25)
    Q3 = df['Value'].quantile(0.75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    print(f"IQR: {IQR:.2f}")
    print(f"IQR bounds: {lower_bound:.2f} - {upper_bound:.2f}")
    return df[(df['Value'] >= lower_bound) & (df['Value'] <= upper_bound)]


def remove_outliers_rolling_mad(df, window=5, thresh=3.5):

    # Rolling Median berechnen
    rolling_median = df['Value'].rolling(window=window, center=True, min_periods=1).median()

    # Absoluter Abstand zum Rolling Median
    abs_deviation = (df['Value'] - rolling_median).abs()

    # Rolling MAD (Median der absoluten Abweichungen)
    rolling_mad = abs_deviation.rolling(window=window, center=True, min_periods=1).median()

    # Skaliere MAD auf vergleichbar mit Standardabweichung (Konstante 1.4826)
    modified_z_score = 0.6745 * abs_deviation / rolling_mad.replace(0, 1e-6)  # Division durch 0 verhindern

    # Punkte behalten, die unter dem Schwellenwert liegen
    filtered_df = df[modified_z_score < thresh]

    print(f"Rolling MAD filter: {len(df) - len(filtered_df)} Ausreißer entfernt")
    return filtered_df

def remove_outliers_mad(df, thresh=2):
    """
    Entfernt Ausreißer basierend auf der Median Absolute Deviation (MAD).

    Args:
        df (pd.DataFrame): DataFrame mit Spalte 'Value'
        thresh (float): Schwellenwert für Ausreißer in MAD-Einheiten

    Returns:
        pd.DataFrame: Gefilterte Daten ohne Ausreißer
    """
    median = df['Value'].median()
    abs_deviation = (df['Value'] - median).abs()
    mad = abs_deviation.median()

    # Skaliere MAD (Konstante 1.4826 macht es vergleichbar zur Standardabweichung bei Normalverteilung)
    modified_z_score = 0.6745 * abs_deviation / (mad if mad else 1e-6)  # Vermeidung Division durch Null

    filtered_df = df[modified_z_score < thresh]

    print(f"MAD filter: {len(df) - len(filtered_df)} Ausreißer entfernt")
    return filtered_df


def ergonomie_kennwerte(df, name):
    print(f"\n--- {name} ---")
    values = df['Value'].values
    zeit = df['Time'].values

    mean = np.mean(values)
    std = np.std(values)
    print(f"Mittelwert: {mean:.2f}°")
    print(f"Standardabweichung: {std:.2f}°")

    gruen = ((values >= 60) & (values <= 100)).sum()
    rot = ((values < 60) | (values > 100)).sum()
    total = len(values)

    print(f"Grün (60–100°): {gruen/total*100:.2f}%")
    print(f"Rot (<60° oder >100°): {rot/total*100:.2f}%")

    krit = (values < 60) | (values > 100)
    dt = np.diff(zeit) if len(zeit) > 1 else np.array([0])
    krit_dauer = 0
    start_idx = None
    for i, k in enumerate(krit):
        if k and start_idx is None:
            start_idx = i
        elif not k and start_idx is not None:
            krit_dauer += np.sum(dt[start_idx:i]) / 1000
            start_idx = None
    if start_idx is not None:
        krit_dauer += np.sum(dt[start_idx:]) / 1000

    print(f"Geschätzte Verweildauer in kritischen Winkeln: {krit_dauer:.2f} s")

    bewegung = np.diff(values)
    zyklen = ((bewegung[:-1] * bewegung[1:]) < 0).sum()
    print(f"Anzahl Haltungswechsel (Bewegungszyklen): {zyklen}")

def plot_ellbogen(df, seite, farbe):
    plt.figure(num=f'Ellbogenwinkel {seite} (ohne Ausreißer) - alle Personen',figsize=(10, 6))
    for person in df['Person'].unique():
        df_person = df[df['Person'] == person].reset_index(drop=True)
        x = np.arange(len(df_person))
        gruen = (df_person['Value'] >= 60) & (df_person['Value'] <= 100)
        rot = (df_person['Value'] < 60) | (df_person['Value'] > 100)
        plt.scatter(x[gruen], df_person['Value'][gruen], color='green', s=20)
        plt.scatter(x[rot], df_person['Value'][rot], color='red', s=20)
        #plt.plot(x, df_person['Value'], linestyle='-', color=farbe, alpha=0.3)
    plt.xlabel('Messwertnummer')
    plt.ylabel('Ellbogenwinkel (°)')
    #plt.title(f'Ellbogenwinkel {seite} (ohne Ausreißer) - alle Personen')
    # Legende nur für die Farben und Grenzen
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', label='Grün (60–100°)', markerfacecolor='green', markersize=5),
        plt.Line2D([0], [0], marker='o', color='w', label='Rot (<60° oder >100°)', markerfacecolor='red', markersize=5)
    ]
    plt.legend(handles=legend_elements)
    plt.grid(True)
    plt.show()

def remove_outliers_hampel(df, column='Value', window_size=10, n_sigmas=3):
    series = df[column]
    rolling_median = series.rolling(window_size, center=True).median()
    diff = np.abs(series - rolling_median)
    mad = diff.rolling(window_size, center=True).median()
    threshold = n_sigmas * 1.4826 * mad  # 1.4826 ist die Skalierung für Normalverteilung
    outlier_mask = diff > threshold
    print(f"Hampel-Filter: {outlier_mask.sum()} Ausreißer erkannt und entfernt")
    return df[~outlier_mask]

def remove_outliers_zscore(df, column='Value', thresh=3.0):
    """
    Entfernt Ausreißer basierend auf dem Z-Score.

    Args:
        df (pd.DataFrame): DataFrame mit der Spalte 'Value'
        column (str): Spaltenname für die Werte
        thresh (float): Z-Score-Schwellenwert für Ausreißer

    Returns:
        pd.DataFrame: Gefilterte Daten ohne Ausreißer
    """
    values = df[column]
    mean = values.mean()
    std = values.std()
    z_scores = (values - mean) / (std if std != 0 else 1e-6)
    outlier_mask = z_scores.abs() > thresh
    print(f"Z-Score-Filter: {outlier_mask.sum()} Ausreißer erkannt und entfernt")
    return df[~outlier_mask]


# Dateien & Personen
dateien = [
    ('armlaengen_niko.csv', 'Niko'),
    ('armlaengen_luka.csv', 'Luka'),
    ('armlaengen_johannes.csv', 'Johannes'),
    ('armlaengen_claudi.csv', 'Claudi')
]

# Alle Daten laden und verbinden
data_all = pd.DataFrame()
for datei, person in dateien:
    df = load_and_prepare_data(datei, person)
    data_all = pd.concat([data_all, df], ignore_index=True)

# Ausreißer pro Person & Ellbogen entfernen
data_clean = pd.DataFrame()
for person in data_all['Person'].unique():
    for seite in ['links', 'rechts']:
        df_part_clean = data_all[(data_all['Person'] == person) & (data_all['Elbogenwinkel Type'] == seite)]
        # Verschiedene thresh-Werte für remove_outliers_mad ausprobieren und Ergebnisse plotten
        # fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=True, sharey=True)
        # axs = axs.flatten()
        # for idx, thresh in enumerate(range(1, 7)):
        #     df_filtered = remove_outliers_mad(df_part_clean, thresh=thresh)
        #     axs[idx].plot(df_part_clean['Value'].values, label='Original', alpha=0.3)
        #     axs[idx].plot(df_filtered['Value'].values, label=f'MAD thresh={thresh}', alpha=0.8)
        #     axs[idx].set_title(f'MAD thresh={thresh}')
        #     axs[idx].legend()
        #     axs[idx].set_ylabel('Winkel (°)')
        # plt.suptitle(f'MAD-Filter Vergleich: {person} - {seite}')
        # plt.tight_layout(rect=[0, 0, 1, 0.96])
        # plt.show()
        # Für die weitere Analyse z.B. thresh=2 verwenden:
        #df_part_clean = remove_outliers_mad(df_part_clean, thresh=2.5)
        #df_part_clean  = remove_outliers_zscore(df_part_clean, column='Value', thresh=3.0)

        #df_part_clean = remove_outliers_iqr(df_part_clean)
        df_part_clean = df_part_clean
        data_clean = pd.concat([data_clean, df_part_clean], ignore_index=True)

# Ergonomiewerte für alle zusammen (links/rechts getrennt)
for seite in ['rechts', 'links']:
    df_seite = data_clean[data_clean['Elbogenwinkel Type'] == seite]
    ergonomie_kennwerte(df_seite, f"Gesamt - Ellbogen {seite.capitalize()}")

# Plots für alle Personen zusammen
plot_ellbogen(data_clean[data_clean['Elbogenwinkel Type']=='rechts'], 'Rechts', 'r')
plot_ellbogen(data_clean[data_clean['Elbogenwinkel Type']=='links'], 'Links', 'g')

def plot_value_distribution(df, seite):
    plt.figure(num=f'Verteilung der Ellbogenwinkel {seite}', figsize=(8, 5))
    values = df['Value']
    plt.hist(values, bins=30, color='skyblue', edgecolor='black', alpha=0.7)
    plt.axvline(60, color='green', linestyle='--', label='60° Grenze')
    plt.axvline(100, color='green', linestyle='--', label='100° Grenze')
    plt.xlabel('Ellbogenwinkel (°)')
    plt.ylabel('Anzahl Messwerte')
    plt.title(f'Verteilung der Ellbogenwinkel ({seite})')
    plt.legend()
    plt.grid(True)
    plt.show()

plot_value_distribution(data_clean[data_clean['Elbogenwinkel Type']=='rechts'], 'Rechts')
plot_value_distribution(data_clean[data_clean['Elbogenwinkel Type']=='links'], 'Links')
