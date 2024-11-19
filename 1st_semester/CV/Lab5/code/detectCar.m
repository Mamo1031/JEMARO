function detectCar(images, template, carName)
% Function to detect a car in a given imagem

    for i = 1:length(images)
        % Processar cada imagem
        processImage(images{i}, template, carName, i);
    end
    
end
