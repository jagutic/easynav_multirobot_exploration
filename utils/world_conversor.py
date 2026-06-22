import xml.etree.ElementTree as ET
import sys

def scale_gazebo_world(input_file, output_file, scale_factor):
    try:
        tree = ET.parse(input_file)
        root = tree.getroot()
    except Exception as e:
        print(f"❌ Error al abrir el archivo: {e}")
        return

    # 1. Escalar las posiciones <pose> (x y z roll pitch yaw)
    for pose in root.iter('pose'):
        if pose.text:
            vals = pose.text.strip().split()
            if len(vals) == 6:
                # Multiplicamos solo x, y, z. Los ángulos (roll, pitch, yaw) se quedan igual
                vals[0] = str(round(float(vals[0]) * scale_factor, 4))
                vals[1] = str(round(float(vals[1]) * scale_factor, 4))
                vals[2] = str(round(float(vals[2]) * scale_factor, 4))
                pose.text = " ".join(vals)

    # 2. Escalar cajas y primitivas <size> (x y z)
    for size in root.iter('size'):
        if size.text:
            vals = size.text.strip().split()
            if len(vals) == 3:
                vals[0] = str(round(float(vals[0]) * scale_factor, 4))
                vals[1] = str(round(float(vals[1]) * scale_factor, 4))
                vals[2] = str(round(float(vals[2]) * scale_factor, 4))
                size.text = " ".join(vals)

    # 3. Escalar mallas personalizadas <scale> (x y z)
    for scale in root.iter('scale'):
        if scale.text:
            vals = scale.text.strip().split()
            if len(vals) == 3:
                vals[0] = str(round(float(vals[0]) * scale_factor, 4))
                vals[1] = str(round(float(vals[1]) * scale_factor, 4))
                vals[2] = str(round(float(vals[2]) * scale_factor, 4))
                scale.text = " ".join(vals)

    # Guardar el nuevo mundo
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"✅ ¡Mundo reescalado con éxito!")
    print(f"📁 Guardado en: {output_file} (Factor: {scale_factor}x)")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Uso incorrecto. Ejecuta:")
        print("python3 scale_world.py <mundo_original.world> <mundo_nuevo.world> <factor_escala>")
        sys.exit(1)

    in_file = sys.argv[1]
    out_file = sys.argv[2]
    factor = float(sys.argv[3])

    scale_gazebo_world(in_file, out_file, factor)
