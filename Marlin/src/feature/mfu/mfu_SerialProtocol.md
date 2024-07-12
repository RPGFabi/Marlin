Startup
=======================

After startup the MFU sends an

- MFU => 'started\n'


Home with Retraction
=======================
If there is no Filamentsensor in use or it is enabled and detects filament the Printer needs to heat up and then Home the MFU with Retraction
After Heating the Hotend to the needed Temperature (Prevent Cold Extrusion) Printer sends
- MFU <= 'H1 E<ExtruderCount>\n'

After Retracting and Homing the MFU answers
- MFU => 'ok\n

Home without Retraction
=======================
If there is an filamentsensor enabled which does not detect an filament the Printer can directly send an Home Command to the MFU. Since there is no Filament loaded no Retraction or Heating is needed
- MFU <= 'H0 E<ExtruderCount>\n'

After Retracting and Homing the MFU answers
- MFU => 'ok\n

Toolchange
=======================
- MFU <= 'T*Toolindex*\n'

Now the MFU unloads the Filament, switches the Tool and preloads the Filament to the Extrudergears
- MFU => 'ok\n'

MFU now fully loads the Filament and prints ok when done => Currently not Needed
- MFU => 'ok\n'

Load Filament
=======================
For loading an Filament the Printer and MFU handle an Toolchange


Unload CurrentFilament
=======================
To Unload the Filament
- MFU <= 'U\n'

to which the MFU answers after it finished
- MFU => 'ok\n'

MFU detects Filamentrunout
=======================
This is sent by the MFU and triggeres an Filamentchange
- MFU => "E1\n"

MFU send Information, that it was reloaded
=======================
- MFU => "Reloaded\n"