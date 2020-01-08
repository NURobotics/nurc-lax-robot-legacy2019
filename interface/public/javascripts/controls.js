let count = 1;

function setColor(btn) {
    let property = document.getElementById(btn);
    if (count == 1){
        property.style.backgroundColor = "white";
        property.style.color = "black";
        count = 0;
    }
    else{
        property.style.backgroundColor = "black";
        property.style.color = "white";
        count = 1;
    }

}

function colorChangePress(btn){
    let property = document.getElementById(btn);
    property.style.backgroundColor = "white";
    property.style.color = "black";
}

function colorChangeRelease(btn){
    let property = document.getElementById(btn);
    property.style.backgroundColor = "black";
    property.style.color = "white";
}

document.onkeydown = function(e) {
    switch (e.keyCode) {
        case 37:
            document.getElementById("left").click();
            break;
        case 38:
            document.getElementById("up").click();
            break;
        case 39:
            document.getElementById("right").click();
            break;
        case 40:
            document.getElementById("down").click();
            break;
        case 49:
            document.getElementById("1").click();
            colorChangePress("1");
            break;
        case 50:
            document.getElementById("2").click();
            colorChangePress("2");
            break;
        case 51:
            document.getElementById("3").click();
            colorChangePress("3");
            break;
        case 52:
            document.getElementById("4").click();
            colorChangePress("4");
            break;
        case 67:
            document.getElementById("speed down").click();
            colorChangePress("speed down");
            break;
        case 88:
            document.getElementById("halt").click();
            colorChangePress("halt");
            break;
        case 90:
            document.getElementById("speed up").click();
            colorChangePress("speed up");
            break;
    }
};

document.onkeyup = function(e) {
    switch (e.keyCode) {
        case 49:
            colorChangeRelease("1");
            break;
        case 50:
            colorChangeRelease("2");
            break;
        case 51:
            colorChangeRelease("3");
            break;
        case 52:
            colorChangeRelease("4");
            break;
        case 67:
            colorChangeRelease("speed down");
            break;
        case 88:
            colorChangeRelease("halt");
            break;
        case 90:
            colorChangeRelease("speed up");
            break;
    }
};