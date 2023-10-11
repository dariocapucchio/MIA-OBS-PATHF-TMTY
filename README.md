# MIA-OBS-PATHF-TMTY
Equipo para el relevamiento de la telemtría.

# Modificacion de codigo para 4 placas
Cada placa tienen un ID de cliente diferente que se define de la linea 40 a la 43 en el archivo main.cpp. Descomentar el que se quiera selleccionar.

```
//#define CLIENT_ID "MIA_TMTY_01"  // ID del cliente (esta placa)
//#define CLIENT_ID "MIA_TMTY_02"  // ID del cliente
//#define CLIENT_ID "MIA_TMTY_03"  // ID del cliente
#define CLIENT_ID "MIA_TMTY_04"  // ID del cliente
```

Tambien hay que modificar en las subscripciones y publicaciones a los topicos que cliente es.

En las lineas 221 a 231

```
// Suscripciones MQTT
  mqttClient.subscribe("MIA_TMTY_04/control/comando");
  mqttClient.subscribe("MIA_TMTY_04/control/pid");
  mqttClient.subscribe("MIA_TMTY_04/control/digital");
  mqttClient.subscribe("MIA_TMTY_04/servicio/fallas_ds_reset");
  mqttClient.subscribe("MIA_TMTY_04/servicio/hw_reset_response");
  mqttClient.subscribe("MIA_TMTY_04/servicio/power_cal");
  mqttClient.subscribe("MIA_TMTY_04/servicio/i_cal");

  mqttClient.publish("MIA_TMTY_04/servicio/hw_reset","R");    // Publico el reset del hw
  mqttClient.loop();
```

En la funcion callback linea 442 a 551 (son muchas, no las puse, hay que buscarlas). Y lo mismo en la funcion enviarDatosMQTT.