
let mainContainerDOM = document.getElementById("main-container");

let newEle = document.createElement("div");
newEle.innerText = "Dynamic div text";
mainContainerDOM.append(newEle);

requestNodeList();

async function requestNodeList() {
    let response = await fetch("http://localhost/node-tool-api/list");
    let content = await response.json();
    mainContainerDOM.appendChild(divWithInnerText(content));
}

async function divWithInnerText(innerText) {
    let newEle = document.createElement("div");
    newEle.innerText = innerText;
    return newEle;
}