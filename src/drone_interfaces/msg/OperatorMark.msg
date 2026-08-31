# Reczne wskazanie celu przez operatora: klikniecie w zamrozona klatke w GUI.
#
# header.stamp = stamp KLATKI, w ktora kliknieto (nie moment klikniecia). To jest
# caly sens zamrazania obrazu: geolokator rzutuje piksel telemetria z dokladnie
# tej chwili, wiec opoznienie podgladu przestaje byc zrodlem bledu.
#
# u, v = piksel w ORYGINALNEJ klatce kamery (nie w przeskalowanym podgladzie).
std_msgs/Header header
uint8 class_id
float32 u
float32 v
