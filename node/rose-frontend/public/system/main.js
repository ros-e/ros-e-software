
let btn_i2c_send = document.getElementById("btn_i2c_send");

let input_i2c_address = document.getElementById("input_i2c_address");
let input_i2c_cmd = document.getElementById("input_i2c_cmd");
let input_i2c_data = document.getElementById("input_i2c_data");

const urlBase = window.location.origin;
// console.log(window.location)

btn_i2c_send.addEventListener("click", function(e){


    let address = input_i2c_address.value;
    let cmd = input_i2c_cmd.value;
    let data = input_i2c_data.value;

    // console.log(`${address} ${cmd} ${data}`)

    let url = new URL("/i2c/transmit", urlBase);
    url.searchParams.append("addr", address);
    url.searchParams.append("cmd", cmd);
    url.searchParams.append("data", data);
    // console.log(url);

    fetch(url, {method: 'GET'})
    .then(res => res.json())
    .then(function(res) {

        console.log('Click was recorded');
        console.log(res)
        //   if(res.ok) {

    //     return;
    //   }
    //   throw new Error('Request failed.');
    })
    .catch(function(error) {
      console.log(error);
    });

});

