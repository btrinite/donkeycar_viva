#> nmcli con add type wifi ifname wlan0 con-name wifi-cristallin ssid WINET
#> nmcli con edit id wifi-cristallin
nmcli> set ipv4.method auto
nmcli> set 802-1x.eap peap
nmcli> set 802-1x.phase2-auth mschapv2
nmcli> set wifi-sec.key-mgmt wpa-eap
nmcli> set 802-1x.identity az01703
nmcli> set 802-1x.password xxxPASSWORDxxx
nmcli> save
nmcli> activate
