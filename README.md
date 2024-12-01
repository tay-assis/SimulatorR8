# Simulador do Processador R8 com MPU Implementada  
Projeto final desenvolvido na disciplina prática de arquitetura de computadores com o objetivo de implementar um co-processador que realiza operações matriciais (MPU).

---

### Passo 1: Desenvolvimento da MPU  
A Unidade de Processamento Matricial (MPU) será desenvolvida como um periférico mapeado em memória.  

#### 1.1. **Descrição Geral da MPU**  
- O tamanho da MPU é de **64 endereços**.  
- Cada endereço comportará um dado de **16 bits**, utilizando o formato **complemento de 2** para representar números inteiros.  

#### 1.2. **Mapa de Memória da MPU**   
- **Região de Comando (Comunicação com a CPU):** Endereços **0 a 15**.  
- **Matriz C[4:4] (Resultado):** Endereços **16 a 31**.  
- **Matriz A[4:4] (Entrada 1):** Endereços **32 a 47**.  
- **Matriz B[4:4] (Entrada 2):** Endereços **48 a 63**.  

#### 1.3. **Comandos Implementados pela MPU**  
A MPU deverá suportar os seguintes comandos:  

1. **Operações Matriciais:**  
   - `add`: Executa `C[4:4] = A[4:4] + B[4:4]`.  
   - `sub`: Executa `C[4:4] = A[4:4] - B[4:4]`.  
   - `mul`: Executa `C[4:4] = A[4:4] * B[4:4]` (produto matricial).  
   - `mac`: Executa `C[4:4] = C[4:4] + A[4:4] * B[4:4]` (produto acumulado).  

2. **Manipulação de Matrizes:**  
   - `fill M, n`: Preenche a matriz `M[4:4]` com o valor inteiro `n`.  
   - `identity M, n`: Define a matriz `M[4:4]` como uma matriz identidade, multiplicada pelo valor `n`.

3. **Leitura e Escrita:**  
   - **Load:** Lê o dado de uma matriz qualquer (A, B ou C) através do endereço passado pela CPU.
   - **Store:**  Escreve o dado de uma matriz qualquer (A,B ou C) através do endereço passado pela CPU.

---

### Passo 2: *(Descreva aqui o segundo passo do projeto)*  
*(Inclua os detalhes do próximo passo conforme necessário.)*  

---

### Passo 3: *(Descreva aqui o terceiro passo do projeto)*  
*(Inclua os detalhes do próximo passo conforme necessário.)*  

---

## Observações importantes:  
O processador R8 foi desenvolvido na **PUC-RS**, e todas os recursos estão do projeto estão disponíveis no site oficial:  
[R8 Processor Core](https://www.inf.pucrs.br/~calazans/research/Projects/R8/R8_Processor_Core.html)
