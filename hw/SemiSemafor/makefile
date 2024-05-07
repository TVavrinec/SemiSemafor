kikit panelize --layout 'grid; rows: 2; cols: 5; space: 5mm;' \
    --tabs annotation \
    --source 'tolerance: 15mm' \
    --cuts 'mousebites; drill: 0.5mm; spacing: 1mm; offset: 0.2mm; prolong: 0.5mm' \
    --framing 'railstb; width: 5mm; space: 2mm;' \
    --tooling '3hole; hoffset: 2.5mm; voffset: 2.5mm; size: 1.152mm' \
    --post 'millradius: 1mm' SemiSemafor.kicad_pcb build/panel.kicad_pcb

kikit fab jlcpcb --assembly --no-drc --schematic SemiSemafor.kicad_sch build/panel.kicad_pcb jlcdata