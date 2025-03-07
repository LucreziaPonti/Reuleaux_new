import h5py

def esplora_file_h5(percorso_file):
    with h5py.File(percorso_file, 'r') as file:
        def esplora_gruppo(gruppo, percorso='/'):
            for nome, elemento in gruppo.items():
                percorso_completo = percorso + nome
                if isinstance(elemento, h5py.Group):
                    print(f"Gruppo: {percorso_completo}")
                    esplora_gruppo(elemento, percorso_completo + '/')
                elif isinstance(elemento, h5py.Dataset):
                    print(f"Dataset: {percorso_completo}, Shape: {elemento.shape}, Dtype: {elemento.dtype}")
                    # Stampa gli attributi del dataset (potrebbero contenere informazioni sul frame)
                    for attr_name, attr_value in elemento.attrs.items():
                        print(f"  Attributo - {attr_name}: {attr_value}")
                else:
                    print(f"Elemento sconosciuto: {percorso_completo}")

        # Esplora la struttura principale del file
        esplora_gruppo(file)

        # Cerca eventuali metadati globali del file (attributi a livello di file)
        print("\nAttributi globali del file:")
        for attr_name, attr_value in file.attrs.items():
            print(f"  {attr_name}: {attr_value}")

# Usa il percorso del tuo file .h5 qui
percorso_file = '/home/oem/catkin_ws/src/tesi_reuleaux_tiago/reuleaux/map_generation/maps/percorso_nuovo_file.h5'
esplora_file_h5(percorso_file)

"""
import h5py

# Sostituisci 'tuo_file.h5' con il percorso del tuo file
file_path = '/home/oem/catkin_ws/src/tesi_reuleaux_tiago/reuleaux/map_generation/maps/tiago_arm_0.15_reachability.h5'

with h5py.File(file_path, 'r') as f:
    # Stampa l'intestazione del file
    print("File header:")
    for key in f.keys():
        print(f"  {key}: {f[key].name}")

    # Stampa la risoluzione
    resolution = f['/Spheres/sphere_dataset'].attrs.get('Resolution', None)
    if resolution is not None:
        print(f"Risoluzione: {resolution}")

    def print_structure(name, obj):
        print(f"{name} (type: {type(obj)})")
        if isinstance(obj, h5py.Dataset):
            print(f"  Dataset shape: {obj.shape}")
            print(f"  Dataset dtype: {obj.dtype}")
            # Stampa gli attributi
            for attr in obj.attrs:
                print(f"    Attribute {attr}: {obj.attrs[attr]}")

    f.visititems(print_structure)
"""