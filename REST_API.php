<?php
/*
REST API for iTRUBEC SigFox monitor based on SONY Spresense

Message Types:
DATA:  $a1 == 1  --> temperature, humidity, atmospherical pressure, sound level
ALERT: $a1 == 2  --> position (latitude & longitude)

SigFox DATA message format
a1::uint:8 t1::uint:8 v1::uint:8 p1::uint:8 s1::uint:8
example: 014B316A65

SigFox DATA Callback Format
http://itrubec.cz/monitor/REST_API.php?key=xxxxxxx&dev={device}&t1={customData#t1}&v1={customData#v1}&p1={customData#p1}&s1={customData#s1}&a1={customData#a1}&rssi={rssi}&seqn={seqNumber}&lat={lat}&lng={lng}&snr={snr}&avgsnr={avgSnr}

GET DATA callback example
http://itrubec.cz/monitor/REST_API.php?key=xxxxxx&dev=xxxxxx&t1=75&v1=49&p1=106&s1=86&a1=1&rssi=-102.00&seqn=4&lat=49.0&lng=17.0&snr=25.83&avgsnr=23.77

SigFox ALERT message format
a1::uint:8 la::float:32 lo::float:32
example:

SigFox ALERT Callback Format
http://itrubec.cz/monitor/REST_API.php?key=xxxxxxx&dev={device}&la={customData#la}&lo={customData#lo}&a1={customData#a1}&rssi={rssi}&seqn={seqNumber}&lat={lat}&lng={lng}&snr={snr}&avgsnr={avgSnr}

GET ALERT callback example
http://itrubec.cz/monitor/REST_API.php?key=xxxxxxx&dev=xxxxxxxx&la=49.21095&lo=16.5760356&a1=2&rssi=-102.00&seqn=4&lat=49.0&lng=17.0&snr=25.83&avgsnr=23.77
*/

$_GET   = filter_input_array(INPUT_GET, FILTER_SANITIZE_STRING);

$servername = "XXXX";   //jméno serveru
$username = "XXXX";       //DB login
$password = "XXXX";//DB psswd
$dbname = "XXXX";       //DB name

// Create connection
$mysqli =  new mysqli($servername, $username, $password, $dbname);

/* check connection */
if ($mysqli->connect_errno) {
    printf("Connect failed: %s\n", $mysqli->connect_error);
    exit();
}

$key = $_GET["key"];
$dev = $_GET["dev"];
$t1 = $_GET["t1"];
$v1 = $_GET["v1"];
$p1 = $_GET["p1"];
$s1 = $_GET["s1"];
$a1 = $_GET["a1"];
$rssi = $_GET["rssi"];
$seqn = $_GET["seqn"];
$lat = $_GET["lat"];
$lng = $_GET["lng"];
$la = $_GET["la"];
$lo = $_GET["lo"];
$snr = $_GET["snr"];
$avgsnr = $_GET["avgsnr"];

$t1 = $t1 - 50;
$p1 = $p1 + 885;

//SELECT ID FROM `monitor_devices` WHERE devkey like "xxxxxx" AND device like "xxxxx"
$sql = "SELECT ID FROM `monitor_devices` WHERE devkey like \"".$key."\" AND device like \"".$dev."\"";
if ($result = $mysqli->query($sql)) {
    printf("%d record found.\n", $result->num_rows);
    $row=mysqli_fetch_row($result);
    $devid = $row[0];
    $result->close();
    $sql = "UPDATE `monitor_devices` SET `lastactivity`=now() WHERE `id`=".$devid;
    echo("<br>");
    echo($sql);
    echo("<br>");
    if ($mysqli->query($sql) === TRUE) {
      printf("Last seen timestamp updated.<br>\n");
    }
}else{
   die("Bad key or bad device!");
}

if ($a1 == 1){  //data call
  $sql = "INSERT INTO `monitor_data`(`devid`, `timestamp`, `temp1`, `humidity1`, `pressure1`, `rssi`, `seqn`, `lat`, `lng`, `snr`, `avgsnr`, `TheftAlert`, `Sound`) VALUES ($devid, now(),$t1,$v1,$p1,$rssi,$seqn,$lat,$lng,$snr,$avgsnr,0,$s1)";
  echo("<br>");
  echo($sql);
  echo("<br>");
  if ($result = $mysqli->query($sql)) {
      echo "OK<br>";
  } else {
      echo "Error" . $sql;
      echo("<br>");
  }
}

if ($a1 == 2){  //Theft Alert call
  //SMS alert!
   $to = "+420XXXXXXXXX@sms.cz.o2.com";
   $subject = "Theft Alert";
   $txt = "Motion detected at beehive #".$dev."\nLat: ".$la."\nLon: ".$lo;
   $headers = 'From: info@XXXX.cz' . "\r\n" .
    'Reply-To: info@XXXX.cz' . "\r\n" .
    'Cc: info@XXXX.cz' . "\r\n" .
    'X-Mailer: PHP/' . phpversion();
   mail($to,$subject,$txt,$headers);

   $sql = "INSERT INTO `monitor_data`(`devid`, `timestamp`, `rssi`, `seqn`, `lat`, `lng`, `snr`, `avgsnr`, `TheftAlert`) VALUES ($devid, now(),$rssi,$seqn,$la,$lo,$snr,$avgsnr,1)";
   echo("<br>");
   echo($sql);
   echo("<br>");
  if ($result = $mysqli->query($sql)) {
      echo "OK<br>";
  } else {
      echo "Error" . $sql;
      echo("<br>");
  }
}

$mysqli->close();

?>