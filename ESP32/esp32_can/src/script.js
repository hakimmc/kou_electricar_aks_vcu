function doGet(e) {
    Logger.log( JSON.stringify(e) ); // Parametreleri görüntüle
    
    var result = "basarili"; // Başarili
    if (e.parameter == "undefined") {
    result = "Hatali parametre!!";
    }
    else {
    var sheet_id = "10VgAlqsbOMK2Gl-8UocOU20k71v38XzRWBmPWTJenk0"; // Buraya google shhet ID"nizi adres satirindan alip yapiştirin
    var sheet = SpreadsheetApp.openById(sheet_id).getActiveSheet(); // ID üzerinden sheed"inize erişiyoruz
    var newRow = sheet.getLastRow()+1;
    var rowData = [];
    rowData[0] = new Date().toLocaleString('tr-TR'); // Zaman damgasi A kolonuna yazildi
    for (var param in e.parameter) {
    Logger.log("In for loop, param="+ param);
    var value = stripQuotes(e.parameter[param]);
    Logger.log(param +":"+ e.parameter[param]);// Ekrana yazdiriyoruz aldiğimiz parametreyi
    switch(param){
    case "yon": //Parametremiz
    rowData[1]= value;//B kolonuna değeri yazdi
    result = "yon değeri B kolonuna yazildi";
    break;
    case "hiz": //Parametremiz
    rowData[2] = value; //C kolonuna değeri yazdi
    result += " hiz değeri C kolonuna yazildi";
    break;
    case "sicaklik": //Parametremiz
    rowData[3]= value;//B kolonuna değeri yazdi
    result = "sicaklik değeri D kolonuna yazildi";
    break;
    case "b1v": //Parametremiz
    rowData[4] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b2v": //Parametremiz
    rowData[5] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b3v": //Parametremiz
    rowData[6] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b4v": //Parametremiz
    rowData[7] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b5v": //Parametremiz
    rowData[8] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b6v": //Parametremiz
    rowData[9] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b7v": //Parametremiz
    rowData[10] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b8v": //Parametremiz
    rowData[11] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b9v": //Parametremiz
    rowData[12] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b10v": //Parametremiz
    rowData[13] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b11v": //Parametremiz
    rowData[14] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b12v": //Parametremiz
    rowData[15] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b13v": //Parametremiz
    rowData[16] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b14v": //Parametremiz
    rowData[17] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b15v": //Parametremiz
    rowData[18] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b16v": //Parametremiz
    rowData[19] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b17v": //Parametremiz
    rowData[20] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b18v": //Parametremiz
    rowData[21] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b19v": //Parametremiz
    rowData[22] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b20v": //Parametremiz
    rowData[23] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b21v": //Parametremiz
    rowData[24] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b22v": //Parametremiz
    rowData[25] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b23v": //Parametremiz
    rowData[26] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    case "b24v": //Parametremiz
    rowData[27] = value; //C kolonuna değeri yazdi
    result += " b1v değeri E-AB kolonuna yazildi";
    break;
    default:
    result = "Parametre hatali!";
    }
    
    }
    Logger.log(JSON.stringify(rowData));
    
    var newRange = sheet.getRange(newRow, 1, 1, rowData.length);
    newRange.setValues([rowData]);
    var cRange = sheet.getRange(2, 1, 1, rowData.length);
    cRange.setValues([rowData]);
    }
    //Sonucu döndüren fonksiyon
    return ContentService.createTextOutput(result);
    }
    
     
    
    function stripQuotes( value ) {
    return value.replace(/^[""]|[""]$/g, "");
    }