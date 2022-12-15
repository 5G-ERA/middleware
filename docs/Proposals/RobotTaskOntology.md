# 5G-ERA PROJECT RESEARCH: Robot task Ontology: 

| Author | Partner | Last edit time |
|--------|---------|----------------|
|    Adrian Lendinez    |     University of Bedfordshire    |       15-12-2022         |


- **Addressed a github issue?** Yes
- (*If not, include the issue definition here:*) ...

- **Github issue Id:** [#86](https://github.com/5G-ERA/middleware/issues/86)

- **System/s affected:** Middleware and reference NetApp.
- **Partners involved:** BED
___

- **Technical proposal specifications:** 

## *Background:*
Need for better task naming and use it instead of GUID taskID to call a task.

## *Technical proposal:*

The task name must be a collection of the following items from these tables: 

```
user command + "-" + high level action family + "-" + Industry
```
Example:
```
clean_table-Manipulation-domestic_service 
```

## 1. Industry scope: 
Different contexts.

| Industry           |         |
|--------------------|---------|
| Chemical           |         |
| Commerce           |         |
| Construction       |         |
| Education          |         |
| Public service     |         |
| Transport          |         |
| Textile            |         |
| Postal and telecom |         |
| Energy             |         |
| Media & Culture    |         |
| Mining             |         |
| Turism and hotel   |         |
| Healthcare         |  |
| forestry           |         |
| food/drink         |         |
| metal production   |         |
| domestic service   |         |

## Types of  high level action family: 

| High level action family           |         |
|--------------------|---------|
| Transport           |         |
| Elaboration           |         |
| Surveilance           |         |
| Construction           |         |
| Manipulation           |         |
| Inspection           |         |
| Manteninance           |         |


## Nature of task:

| Nature      |         |
|--------------------|---------|
| Repetitive           |         |
| Colaborative           |         |
| Dangerous           |         |
| Critical           |         |
| Delicate           |         |
